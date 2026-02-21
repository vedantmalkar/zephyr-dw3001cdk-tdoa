#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define ROLE_TRANSMITTER 0   // set 1 for TX, 0 for RX

static dwt_config_t uwb_cfg = {
    .chan           = 9,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC          = DWT_PAC8,
    .txCode         = 9,
    .rxCode         = 9,
    .sfdType        = DWT_SFD_DW_8,
    .dataRate       = DWT_BR_6M8,
    .phrMode        = DWT_PHRMODE_STD,
    .phrRate        = DWT_PHRRATE_STD,
    .sfdTO          = (129 + 8 - 8),
    .stsMode        = DWT_STS_MODE_OFF,
    .stsLength      = DWT_STS_LEN_64,
    .pdoaMode       = DWT_PDOA_M0,
};

static uint8_t tx_msg[] = {0xAB, 0xCD, 0x00};

static int uwb_init(void){

    dw_device_init();

    dw3000_hw_wakeup_pin_low();
    Sleep(5);

    port_set_dw_ic_spi_slowrate();

    if(dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS){
        LOG_ERR("CAN NOT FIND CHIP DRIVER");
        return -1;
    }

    port_set_dw_ic_spi_fastrate();

    if(dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS){
        LOG_ERR("INITIALISATION FAILED");
        return -1;
    }

    if(dwt_configure(&uwb_cfg) != DWT_SUCCESS){
        LOG_ERR("RADIO CONFIG FAILED");
        return -1;
    }

    dwt_setrxantennadelay(16385);
    dwt_settxantennadelay(16385);

    return 0;
}

#if ROLE_TRANSMITTER

static void tx_loop(void){

    uint8_t seq = 0;

    uint8_t ts_raw[5];     // ADDED
    uint64_t ts;           // ADDED

    while(1){

        tx_msg[2] = seq;

        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
        dwt_writetxfctrl(sizeof(tx_msg) + FCS_LEN, 0, 0);

        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while(!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK));

        // -------------------------
        // ADDED PART: READ TX TIMESTAMP
        // -------------------------

        dwt_readtxtimestamp(ts_raw);

        ts = 0;

        for(int i = 4; i >= 0; i--){
            ts = (ts << 8) | ts_raw[i];
        }

        LOG_INF("TX data send %d  ts=0x%08x%08x",
            seq,
            (uint32_t)(ts >> 32),
            (uint32_t)(ts & 0xFFFFFFFF));

        // -------------------------

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        seq++;

        Sleep(500);
    }
}

#else

static void rx_loop(void){

    uint8_t rx_buffer[128];
    uint8_t ts_raw[5];

    while(1){

        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;

        while(!((status = dwt_readsysstatuslo()) &
              (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

        if(status & DWT_INT_RXFCG_BIT_MASK){

            uint16_t len = dwt_getframelength();

            dwt_readrxdata(rx_buffer, len - FCS_LEN, 0);

            dwt_readrxtimestamp(ts_raw);

            uint64_t ts = 0;

            for(int i = 4; i >= 0; i--){
                ts = (ts << 8) | ts_raw[i];
            }

            LOG_INF("RX [%d] ts=0x%08x%08x len=%d data: %02X %02X %02X",
                rx_buffer[2],
                (uint32_t)(ts >> 32),
                (uint32_t)(ts & 0xFFFFFFFF),
                len - FCS_LEN,
                rx_buffer[0],
                rx_buffer[1],
                rx_buffer[2]);
        }
        else{
            LOG_ERR("Receiving Error");
        }

        dwt_writesysstatuslo(
            DWT_INT_RXFCG_BIT_MASK |
            SYS_STATUS_ALL_RX_ERR |
            SYS_STATUS_ALL_RX_TO);
    }
}

#endif


int main(void){

    LOG_INF("Starting up board");

    if(uwb_init() != 0){

        LOG_ERR("UWB STARTUP FAILED");

        return -1;
    }

    LOG_INF("STARTUP SUCCESS");

#if ROLE_TRANSMITTER

    tx_loop();

#else

    rx_loop();

#endif

    return 0;
}