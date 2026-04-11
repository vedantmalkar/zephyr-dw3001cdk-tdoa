/* MASTER ANCHOR */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(tdoa_master, LOG_LEVEL_INF);

/* ================= USER CONFIG ================= */

#define NODE_ID 1
#define ANT_DLY 26194

#define MSG_SYNC 0x10
#define SYNC_PERIOD_MS 100
#define UUS_TO_DWT_TIME 63898

/* =============================================== */

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

/* ================= TIMESTAMP HELPER ================= */

static uint64_t get_tx_ts(void)
{
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);

    uint64_t val = 0;

    for(int i=4;i>=0;i--)
        val = (val<<8) | ts[i];

    return val;
}

/* ================= UWB INIT ================= */

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

/* ================= MASTER LOOP ================= */

static void master_sync_loop(void)
{
    uint8_t sync_msg[16];

    uint16_t seq = 0;

    uint64_t next_tx_time;
    uint64_t last_tx_time = 0;

    uint8_t ts[5];
    uint64_t now = 0;

    dwt_readsystime(ts);

    for(int i=4;i>=0;i--)
        now = (now<<8) | ts[i];

    next_tx_time = now + (SYNC_PERIOD_MS * 1000 * UUS_TO_DWT_TIME);

    while(1)
    {
        sync_msg[0] = MSG_SYNC;
        sync_msg[1] = seq;
        sync_msg[2] = NODE_ID;

        for(int i=0;i<5;i++)
            sync_msg[3+i] = (last_tx_time>>(8*i));

        dwt_writetxdata(8, sync_msg, 0);
        dwt_writetxfctrl(8+FCS_LEN,0,0);

        dwt_setdelayedtrxtime(next_tx_time >> 8);
        dwt_starttx(DWT_START_TX_DELAYED);

        while(!(dwt_readsysstatuslo() &
               DWT_INT_TXFRS_BIT_MASK));

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        last_tx_time = get_tx_ts();

        /* CSV output */

        LOG_INF("MASTER,%u,%llu", seq, last_tx_time);

        next_tx_time += (SYNC_PERIOD_MS * 1000 * UUS_TO_DWT_TIME);

        seq++;
    }
}

int main(void)
{
    LOG_INF("TDOA Master Anchor Start");

    if(uwb_init()!=0)
    {
        LOG_ERR("Init failed");
        return -1;
    }

    master_sync_loop();

    return 0;
}