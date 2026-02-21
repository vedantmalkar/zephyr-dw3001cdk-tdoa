#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define ROLE_INITIATOR 0

#define ANT_DLY              16385
#define SPEED_OF_LIGHT       299702547.0

#define RESP_DELAY_UUS       1000
#define UUS_TO_DWT_TIME      65536

#define MSG_POLL  0x01
#define MSG_RESP  0x02

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

static int uwb_init(void)
{
    dw_device_init();
    dw3000_hw_wakeup_pin_low();
    Sleep(5);
    port_set_dw_ic_spi_slowrate();

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        LOG_ERR("dwt_probe failed");
        return -1;
    }

    port_set_dw_ic_spi_fastrate();

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        LOG_ERR("dwt_initialise failed");
        return -1;
    }

    if (dwt_configure(&uwb_cfg) != DWT_SUCCESS) {
        LOG_ERR("dwt_configure failed");
        return -1;
    }

    dwt_setrxantennadelay(ANT_DLY);
    dwt_settxantennadelay(ANT_DLY);

    return 0;
}

static uint64_t read_ts(void (*read_fn)(uint8_t *))
{
    uint8_t raw[5];
    read_fn(raw);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | raw[i];
    }
    return ts;
}

#if ROLE_INITIATOR
static void initiator_loop(void)
{
    uint8_t seq = 0;
    uint8_t poll_msg[4] = {0xAB, 0xCD, MSG_POLL, 0};
    uint8_t resp_buf[32];

    LOG_INF("Role: INITIATOR");

    while (1) {
        poll_msg[3] = seq;

        dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
        dwt_writetxfctrl(sizeof(poll_msg) + FCS_LEN, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK));
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        uint64_t t1 = read_ts(dwt_readtxtimestamp);

        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        uint32_t status;
        while (!((status = dwt_readsysstatuslo()) &
                 (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

        if (status & DWT_INT_RXFCG_BIT_MASK) {
            uint16_t len = dwt_getframelength();
            dwt_readrxdata(resp_buf, len - FCS_LEN, 0);

            uint64_t t4 = read_ts(dwt_readrxtimestamp);

            uint64_t t2 = 0, t3 = 0;
            for (int i = 4; i >= 0; i--) {
                t2 = (t2 << 8) | resp_buf[4 + i];
                t3 = (t3 << 8) | resp_buf[9 + i];
            }

            double round_trip  = (double)(t4 - t1);
            double reply_time  = (double)(t3 - t2);
            double tof         = ((round_trip - reply_time) / 2.0) * DWT_TIME_UNITS;
            double distance    = tof * SPEED_OF_LIGHT;

            LOG_INF("SEQ %d  dist=%.3f m  (t1=%llu t2=%llu t3=%llu t4=%llu)",
                    seq, distance, t1, t2, t3, t4);
        } else {
            LOG_WRN("No response received");
        }

        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);

        seq++;
        Sleep(500);
    }
}

#else
static void responder_loop(void)
{
    uint8_t rx_buf[32];

    LOG_INF("Role: RESPONDER");

    while (1) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        uint32_t status;
        while (!((status = dwt_readsysstatuslo()) &
                 (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

        if (status & DWT_INT_RXFCG_BIT_MASK) {
            uint16_t len = dwt_getframelength();
            dwt_readrxdata(rx_buf, len - FCS_LEN, 0);

            if (rx_buf[2] != MSG_POLL) {
                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
                continue;
            }

            uint64_t t2 = read_ts(dwt_readrxtimestamp);

            uint32_t resp_tx_time = (t2 + (RESP_DELAY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            uint64_t t3 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + ANT_DLY;

            uint8_t resp_msg[14];
            resp_msg[0] = 0xAB;
            resp_msg[1] = 0xCD;
            resp_msg[2] = MSG_RESP;
            resp_msg[3] = rx_buf[3];
            for (int i = 0; i < 5; i++) resp_msg[4 + i] = (t2 >> (8 * i)) & 0xFF;
            for (int i = 0; i < 5; i++) resp_msg[9 + i] = (t3 >> (8 * i)) & 0xFF;

            dwt_writetxdata(sizeof(resp_msg), resp_msg, 0);
            dwt_writetxfctrl(sizeof(resp_msg) + FCS_LEN, 0, 0);
            if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
                LOG_WRN("Delayed TX failed — too late");
            }
            while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK));
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

            LOG_INF("RESP sent  seq=%d", rx_buf[3]);
        } else {
            LOG_WRN("RX error");
        }

        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
    }
}
#endif

int main(void)
{
    LOG_INF("Starting up board");

    if (uwb_init() != 0) {
        LOG_ERR("UWB init failed");
        return -1;
    }

    LOG_INF("UWB ready");

#if ROLE_INITIATOR
    initiator_loop();
#else
    responder_loop();
#endif

    return 0;
<<<<<<< HEAD
}
=======
}
>>>>>>> 7c3140ce492850de624226ea28de4f5530dc41f3
