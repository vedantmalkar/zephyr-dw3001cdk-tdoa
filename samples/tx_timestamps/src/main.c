/*
 * tx_timestamps.c - Transmit blinks and print the 40-bit TX timestamp for each.
 *
 * Self-contained: no second board needed.
 * Sends a 3-byte blink every TX_INTERVAL_MS and logs the DW3000 TX timestamp.
 * Use these timestamps to verify the hardware clock is running and to
 * cross-check tick rate against Zephyr wall time.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(tx_ts, LOG_LEVEL_INF);

#define TX_INTERVAL_MS  100

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
        LOG_ERR("CAN NOT FIND CHIP DRIVER");
        return -1;
    }

    port_set_dw_ic_spi_fastrate();

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        LOG_ERR("INITIALISATION FAILED");
        return -1;
    }

    if (dwt_configure(&uwb_cfg) != DWT_SUCCESS) {
        LOG_ERR("RADIO CONFIG FAILED");
        return -1;
    }

    dwt_setrxantennadelay(16385);
    dwt_settxantennadelay(16385);

    return 0;
}

int main(void)
{
    LOG_INF("tx_timestamps: starting");

    if (uwb_init() != 0) {
        LOG_ERR("UWB STARTUP FAILED");
        return -1;
    }

    LOG_INF("UWB init OK");

    uint8_t msg[] = {0xAB, 0xCD, 0x00};
    uint8_t ts_raw[5];
    uint8_t seq = 0;

    while (1) {
        msg[2] = seq;

        dwt_writetxdata(sizeof(msg), msg, 0);
        dwt_writetxfctrl(sizeof(msg) + FCS_LEN, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) {}

        dwt_readtxtimestamp(ts_raw);
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        uint64_t ts = 0;
        for (int i = 4; i >= 0; i--) {
            ts = (ts << 8) | ts_raw[i];
        }

        LOG_INF("seq=%u  ts=0x%08X%08X", seq,
                (uint32_t)(ts >> 32),
                (uint32_t)(ts & 0xFFFFFFFF));

        seq++;
        k_msleep(TX_INTERVAL_MS);
    }

    return 0;
}
