/*
 * clock_drift.c — DW3000 clock drift measurement sample
 *
 * DRIFT_MODE 0: Single-chip self-test (no second board needed)
 *               Compares DW3000 tick counter against Zephyr wall clock.
 *
 * DRIFT_MODE 1: TX blinks (pair with a board flashed DRIFT_MODE 2)
 *               Sends {0xAB, 0xCD, seq} every 100 ms.
 *
 * DRIFT_MODE 2: RX drift measurement (pair with DRIFT_MODE 1 board)
 *               Reads carrier integrator + clock offset per blink,
 *               averages over DRIFT_WINDOW packets, logs cumulative drift.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(clock_drift, LOG_LEVEL_INF);

/* ── Mode select ─────────────────────────────────────────────────────────── */
#define DRIFT_MODE  0
/* 0 = single-chip self-test  (no second board)
 * 1 = TX blinks              (paired with mode 2)
 * 2 = RX drift measurement   (reads CI/CO from received blinks) */

/* ── Clock constants ─────────────────────────────────────────────────────── */
/* DW3000: 499.2 MHz × 128 = 63,897,600,000 ticks / second */
#define DW_TICKS_PER_SEC        63897600000ULL
#define DW_TICKS_PER_MS         (DW_TICKS_PER_SEC / 1000ULL)

/* Carrier integrator → Hz conversion */
#define FREQ_OFFSET_MULTIPLIER      (998.4e6 / 2.0 / 1024.0 / 131072.0)
/* Hz → ppm for channel 9 (centre 7987.2 MHz) */
#define HERTZ_TO_PPM_MULTIPLIER_CH9 (-1.0e6 / 7987.2e6)

/* 40-bit counter rollover mask */
#define TS_MASK_40BIT   0xFFFFFFFFFFULL

/* ── Mode-specific tunables ───────────────────────────────────────────────── */
#define SELF_TEST_INTERVAL_MS   1000
#define BLINK_INTERVAL_MS       100
#define DRIFT_WINDOW            16      /* packets to average before logging */

/* ── UWB config (identical to simple_rx_tx.c) ────────────────────────────── */
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

/* ── Init (verbatim from simple_rx_tx.c) ─────────────────────────────────── */
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

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/* Read 40-bit RX timestamp from 5-byte little-endian buffer */
static uint64_t read_rx_ts(void)
{
    uint8_t buf[5];
    dwt_readrxtimestamp(buf);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | buf[i];
    }
    return ts;
}

/* ── DRIFT_MODE 0: single-chip self-test ────────────────────────────────── */
#if DRIFT_MODE == 0

static void self_test_loop(void)
{
    uint32_t sys_hi_first = 0;
    uint32_t wall_first   = 0;
    bool     first        = true;

    LOG_INF("[SELF] Starting single-chip self-test (interval %d ms)",
            SELF_TEST_INTERVAL_MS);

    while (1) {
        k_msleep(SELF_TEST_INTERVAL_MS);

        uint32_t sys_hi  = dwt_readsystimestamphi32();
        uint32_t wall_ms = k_uptime_get_32();

        if (first) {
            sys_hi_first = sys_hi;
            wall_first   = wall_ms;
            first        = false;
            LOG_INF("[SELF] Reference captured: sys_hi=0x%08X  wall=%u ms",
                    sys_hi_first, wall_first);
            continue;
        }

        uint32_t elapsed_wall_ms = wall_ms - wall_first;

        /*
         * dwt_readsystimestamphi32() returns the high 32 bits of the 40-bit
         * counter.  Shifting left by 8 maps it back into the 40-bit space,
         * giving ~4 ms resolution — sufficient for ppm estimation over seconds.
         */
        uint64_t ticks_actual   = ((uint64_t)(sys_hi - sys_hi_first)) << 8;
        uint64_t ticks_expected = (uint64_t)elapsed_wall_ms * DW_TICKS_PER_MS;

        double drift_ppm = 0.0;
        if (ticks_expected > 0) {
            drift_ppm = ((double)ticks_actual - (double)ticks_expected)
                        / (double)ticks_expected * 1e6;
        }

        /* Log with integer arithmetic for ticks (no float format specifier
         * issues on all Zephyr configs) then drift as fixed-point. */
        int32_t drift_ppm_int  = (int32_t)drift_ppm;
        int32_t drift_ppm_frac = (int32_t)((drift_ppm - drift_ppm_int) * 100);
        if (drift_ppm_frac < 0) {
            drift_ppm_frac = -drift_ppm_frac;
        }

        LOG_INF("[SELF] elapsed=%u ms  ticks_actual=%u%08u  ticks_expected=%u%08u  drift=%d.%02d ppm",
                elapsed_wall_ms,
                (uint32_t)(ticks_actual   >> 32), (uint32_t)(ticks_actual   & 0xFFFFFFFF),
                (uint32_t)(ticks_expected >> 32), (uint32_t)(ticks_expected & 0xFFFFFFFF),
                drift_ppm_int, drift_ppm_frac);
    }
}

/* ── DRIFT_MODE 1: TX blinks ─────────────────────────────────────────────── */
#elif DRIFT_MODE == 1

static uint8_t tx_msg[] = {0xAB, 0xCD, 0x00};

static void tx_loop(void)
{
    uint8_t  seq = 0;
    uint8_t  ts_raw[5];
    uint64_t ts;

    LOG_INF("[TX] Starting blink TX (interval %d ms)", BLINK_INTERVAL_MS);

    while (1) {
        tx_msg[2] = seq;

        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
        dwt_writetxfctrl(sizeof(tx_msg) + FCS_LEN, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) {}

        dwt_readtxtimestamp(ts_raw);
        ts = 0;
        for (int i = 4; i >= 0; i--) {
            ts = (ts << 8) | ts_raw[i];
        }

        LOG_INF("[TX] seq=%u  ts=0x%08X%08X",
                seq,
                (uint32_t)(ts >> 32),
                (uint32_t)(ts & 0xFFFFFFFF));

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        seq++;
        k_msleep(BLINK_INTERVAL_MS);
    }
}

/* ── DRIFT_MODE 2: RX drift measurement ─────────────────────────────────── */
#elif DRIFT_MODE == 2

static void rx_drift_loop(void)
{
    uint8_t  rx_buffer[128];
    uint32_t status;

    /* Rolling window accumulators */
    double   sum_ci   = 0.0;
    double   sum_co   = 0.0;
    int      win_cnt  = 0;

    /* Long-term cumulative drift state */
    uint64_t ts_first    = 0;
    uint32_t wall_first  = 0;
    bool     have_first  = false;

    LOG_INF("[DRIFT] Starting RX drift measurement (window %d pkts)", DRIFT_WINDOW);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (1) {
        while (!((status = dwt_readsysstatuslo()) &
                 (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {}

        if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
            /* RX error — clear, re-enable, continue */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }

        /* Good frame: read timestamp first */
        uint64_t rx_ts = read_rx_ts();
        uint32_t wall_now = k_uptime_get_32();

        uint16_t len = dwt_getframelength();
        dwt_readrxdata(rx_buffer, len - FCS_LEN, 0);

        /* Carrier integrator → ppm */
        int32_t ci_raw = dwt_readcarrierintegrator();
        double ppm_ci  = (double)ci_raw
                         * FREQ_OFFSET_MULTIPLIER
                         * HERTZ_TO_PPM_MULTIPLIER_CH9;

        /* Clock offset → ppm */
        int16_t co_raw = dwt_readclockoffset();
        double ppm_co  = (double)co_raw / 16.0;

        /* Clear status */
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK |
                             SYS_STATUS_ALL_RX_ERR  |
                             SYS_STATUS_ALL_RX_TO);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* First-packet reference for cumulative drift */
        if (!have_first) {
            ts_first   = rx_ts;
            wall_first = wall_now;
            have_first = true;
        }

        /* Accumulate rolling window */
        sum_ci  += ppm_ci;
        sum_co  += ppm_co;
        win_cnt++;

        uint8_t seq = (len >= 3) ? rx_buffer[2] : 0xFF;

        if (win_cnt >= DRIFT_WINDOW) {
            double avg_ci = sum_ci / (double)DRIFT_WINDOW;
            double avg_co = sum_co / (double)DRIFT_WINDOW;

            /* 40-bit rollover-safe elapsed ticks */
            uint64_t elapsed_ticks = (rx_ts - ts_first) & TS_MASK_40BIT;
            uint32_t elapsed_wall_ms = wall_now - wall_first;

            double cumul_ppm = 0.0;
            if (elapsed_wall_ms > 0) {
                double ticks_per_ms_actual =
                    (double)elapsed_ticks / (double)elapsed_wall_ms;
                cumul_ppm = (ticks_per_ms_actual / (double)DW_TICKS_PER_MS - 1.0) * 1e6;
            }

            /* Format ppm values as integer + fractional for portability */
            int32_t ci_int  = (int32_t)avg_ci;
            int32_t ci_frac = (int32_t)((avg_ci - ci_int) * 100);
            int32_t co_int  = (int32_t)avg_co;
            int32_t co_frac = (int32_t)((avg_co - co_int) * 100);
            int32_t cu_int  = (int32_t)cumul_ppm;
            int32_t cu_frac = (int32_t)((cumul_ppm - cu_int) * 100);

            if (ci_frac < 0) { ci_frac = -ci_frac; }
            if (co_frac < 0) { co_frac = -co_frac; }
            if (cu_frac < 0) { cu_frac = -cu_frac; }

            LOG_INF("[DRIFT] seq=%u  CI=%d.%02d ppm  CO=%d.%02d ppm  cumul=%d.%02d ppm",
                    seq,
                    ci_int, ci_frac,
                    co_int, co_frac,
                    cu_int, cu_frac);

            /* Reset rolling accumulators */
            sum_ci  = 0.0;
            sum_co  = 0.0;
            win_cnt = 0;
        }
    }
}

#else
#error "DRIFT_MODE must be 0, 1, or 2"
#endif

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    LOG_INF("clock_drift sample  (DRIFT_MODE=%d)", DRIFT_MODE);

    if (uwb_init() != 0) {
        LOG_ERR("UWB STARTUP FAILED");
        return -1;
    }

    LOG_INF("UWB init OK");

#if DRIFT_MODE == 0
    self_test_loop();
#elif DRIFT_MODE == 1
    tx_loop();
#elif DRIFT_MODE == 2
    rx_drift_loop();
#endif

    return 0;
}
