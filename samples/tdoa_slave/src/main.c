/* SLAVE ANCHOR */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(tdoa_slave, LOG_LEVEL_INF);

#define NODE_ID 2
#define ANT_DLY 26194
#define MSG_SYNC 0x10
#define MSG_BLINK 0x20

#define MASK40 0xFFFFFFFFFFULL
#define HALF40 (1LL<<39)
#define FULL40 (1LL<<40)

static dwt_config_t config = {
    .chan = 9,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC = DWT_PAC8,
    .txCode = 9,
    .rxCode = 9,
    .sfdType = DWT_SFD_DW_8,
    .dataRate = DWT_BR_6M8,
    .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD,
    .sfdTO = (129 + 8 - 8),
    .stsMode = DWT_STS_MODE_OFF,
    .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0,
};

/* RX timestamp */

static uint64_t get_rx_ts(void)
{
    uint8_t ts[5];
    dwt_readrxtimestamp(ts);

    uint64_t val = 0;

    for(int i=4;i>=0;i--)
        val = (val<<8) | ts[i];

    return val;
}

/* INIT */

static int uwb_init(void)
{
    dw_device_init();
    dw3000_hw_wakeup_pin_low();
    Sleep(5);

    port_set_dw_ic_spi_slowrate();

    if(dwt_probe((struct dwt_probe_s*)&dw3000_probe_interf)!=DWT_SUCCESS)
        return -1;

    port_set_dw_ic_spi_fastrate();

    if(dwt_initialise(DWT_DW_INIT)!=DWT_SUCCESS)
        return -1;

    if(dwt_configure(&config)!=DWT_SUCCESS)
        return -1;

    dwt_settxantennadelay(ANT_DLY);
    dwt_setrxantennadelay(ANT_DLY);

    return 0;
}
/* Time stamp converter for message receiving*/
/* Convert local DW time → master time */
static uint64_t local_to_master(uint64_t local,
                               int64_t offset,
                               double drift)
{
    double corrected =
        ((double)local - (double)offset) / drift;

    return ((uint64_t)corrected) & MASK40;
}
/* SYNC RECEIVER */
static void slave_loop(void)
{
    uint8_t rx_buf[32];

    uint64_t prev_tx = 0;
    uint64_t prev_rx = 0;

    int64_t offset = 0;
    double drift = 1.0;

    while(1)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;

        while(!((status=dwt_readsysstatuslo()) &
              (DWT_INT_RXFCG_BIT_MASK |
               SYS_STATUS_ALL_RX_ERR)));

        if(!(status & DWT_INT_RXFCG_BIT_MASK))
        {
            dwt_writesysstatuslo(
                DWT_INT_RXFCG_BIT_MASK |
                SYS_STATUS_ALL_RX_ERR);
            continue;
        }

        uint16_t len = dwt_getframelength();
        dwt_readrxdata(rx_buf,len-FCS_LEN,0);

        uint64_t rx_time = get_rx_ts();

        /* ---------------- BLINK ---------------- */

        if(rx_buf[0]==MSG_BLINK)
        {
            if(prev_tx == 0)
            {
                LOG_WRN("BLINK ignored: sync not ready");
            }
            else
            {
                uint64_t master_time =
                    local_to_master(rx_time, offset, drift);

                LOG_INF("BLINK,%llu,%llu",
                        rx_time,
                        master_time);
            }
        }

        /* ---------------- SYNC ---------------- */

        if(rx_buf[0]==MSG_SYNC)
        {
            uint8_t seq = rx_buf[1];

            uint64_t tx_time = 0;

            for(int i=0;i<5;i++)
                tx_time |= ((uint64_t)rx_buf[3+i])<<(8*i);

            int64_t diff = (int64_t)((rx_time - tx_time) & MASK40);

            if(diff > HALF40)
                diff -= FULL40;

            offset = diff;

            if(prev_tx != 0)
            {
                uint64_t master_dt = (tx_time - prev_tx) & MASK40;
                uint64_t slave_dt  = (rx_time - prev_rx) & MASK40;

                if(slave_dt != 0)
                    drift = (double)master_dt / (double)slave_dt;
            }

            double corrected =
                ((double)rx_time - (double)offset) / drift;

            LOG_INF("SYNC,%u,%llu,%llu,%lld,%.9f,%.0f",
                    seq,
                    tx_time,
                    rx_time,
                    offset,
                    drift,
                    corrected);

            prev_tx = tx_time;
            prev_rx = rx_time;
        }

        dwt_writesysstatuslo(
            DWT_INT_RXFCG_BIT_MASK |
            SYS_STATUS_ALL_RX_ERR);
    }
}

int main(void)
{
    LOG_INF("TDOA Slave Anchor Start");

    if(uwb_init()!=0)
    {
        LOG_ERR("Init failed");
        return -1;
    }

    slave_loop();

    return 0;
}