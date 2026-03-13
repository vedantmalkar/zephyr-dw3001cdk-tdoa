#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(clock_drift, LOG_LEVEL_INF);

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

static int uwb_init(void)
{
    dw_device_init();
    dw3000_hw_wakeup_pin_low();
    Sleep(5);

    port_set_dw_ic_spi_slowrate();

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS)
        return -1;

    port_set_dw_ic_spi_fastrate();

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
        return -1;

    if (dwt_configure(&config) != DWT_SUCCESS)
        return -1;

    return 0;
}

/* Print current DW3000 system time as 10-char hex string (MSB first) */
static void print_sys_time(void)
{
    uint8_t ts[5];
    dwt_readsystime(ts);
    /* ts[0] = LSB, ts[4] = MSB */
    printk("TS:%02x%02x%02x%02x%02x\n",
           ts[4], ts[3], ts[2], ts[1], ts[0]);
}

int main(void)
{
    LOG_INF("Clock Drift Firmware");

    if (uwb_init() != 0) {
        LOG_ERR("UWB init failed");
        return -1;
    }

    console_init();
    LOG_INF("Ready. Send 'r' to read timestamp.");

    while (1) {
        char c = console_getchar();
        if (c == 'r') {
            print_sys_time();
        }
    }

    return 0;
}
