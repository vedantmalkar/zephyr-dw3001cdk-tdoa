/*
 * self_clock_drift.c — Single-board clock drift measurement via TX timestamps.
 *
 * No second board needed.
 * Sends a probe packet every INTERVAL_MS, reads the 40-bit DW3000 TX timestamp,
 * and computes per-interval drift (ppm) by comparing consecutive packet timestamps
 * against Zephyr wall time.
 *
 * Uses consecutive-pair comparison (not cumulative) to avoid the 40-bit counter
 * rollover that occurs every ~17.2 s.
 *
 * Expected output (~1–5 ppm typical for a healthy crystal):
 *   [DRIFT] interval=1009 ms  ticks=64462984192  expected=64462278000  drift=10.96 ppm
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(self_drift, LOG_LEVEL_INF);

/* Interval between probe packets (must be << 17200 ms to avoid 40-bit rollover) */
#define INTERVAL_MS     1000

/* DW3000: 499.2 MHz × 128 = 63,897,600,000 ticks/second = 63,897,600 ticks/ms */
#define DW_TICKS_PER_MS  63897600ULL

/* 40-bit counter mask (rollover safety for consecutive pairs) */
#define TS_MASK_40BIT    0xFFFFFFFFFFULL

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

/* Send one packet and return its 40-bit TX timestamp */
static uint64_t send_probe(uint8_t seq)
{
    uint8_t msg[] = {0xAA, 0x55, seq};
    uint8_t ts_raw[5];

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
    return ts;
}

int main(void)
{
    LOG_INF("self_clock_drift: starting");

    if (uwb_init() != 0) {
        LOG_ERR("UWB STARTUP FAILED");
        return -1;
    }

    LOG_INF("UWB init OK");

    uint8_t  seq      = 0;
    uint64_t ts_prev  = send_probe(seq++);
    uint32_t wall_prev = k_uptime_get_32();

    LOG_INF("[DRIFT] Reference packet sent, measuring...");

    while (1) {
        k_msleep(INTERVAL_MS);

        uint64_t ts_now    = send_probe(seq++);
        uint32_t wall_now  = k_uptime_get_32();

        /*
         * Consecutive-pair comparison: interval is 1 s << 17.2 s rollover period,
         * so (ts_now - ts_prev) is always positive and < 2^40.
         */
        uint32_t wall_delta   = wall_now  - wall_prev;
        uint64_t tick_delta   = (ts_now   - ts_prev) & TS_MASK_40BIT;
        uint64_t tick_expected = (uint64_t)wall_delta * DW_TICKS_PER_MS;

        double drift_ppm = 0.0;
        if (tick_expected > 0) {
            drift_ppm = ((double)tick_delta - (double)tick_expected)
                        / (double)tick_expected * 1e6;
        }

        /*
         * Print tick counts split as "billions" + remainder so the decimal
         * representation is correct (avoids the %u%08u concatenation bug).
         *
         * tick_delta ≈ 63.9 billion for a 1 s interval, so:
         *   billions  = tick_delta / 1,000,000,000  (0–63 for 1 s)
         *   remainder = tick_delta % 1,000,000,000  (always 9 digits)
         */
        uint32_t tick_b = (uint32_t)(tick_delta    / 1000000000ULL);
        uint32_t tick_r = (uint32_t)(tick_delta    % 1000000000ULL);
        uint32_t exp_b  = (uint32_t)(tick_expected / 1000000000ULL);
        uint32_t exp_r  = (uint32_t)(tick_expected % 1000000000ULL);

        int32_t d_int  = (int32_t)drift_ppm;
        int32_t d_frac = (int32_t)((drift_ppm - (double)d_int) * 100);
        if (d_frac < 0) { d_frac = -d_frac; }

        LOG_INF("[DRIFT] interval=%u ms  ticks=%u%09u  expected=%u%09u  drift=%d.%02d ppm",
                wall_delta,
                tick_b, tick_r,
                exp_b,  exp_r,
                d_int,  d_frac);

        ts_prev   = ts_now;
        wall_prev = wall_now;
    }

    return 0;
}
