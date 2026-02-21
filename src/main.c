#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define ROLE_TRANSMITTER 1

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

int main(void){
    LOG_INF("Starting up board");
    if(uwb_init() != 0){
        LOG_ERR("UWB STARTUP FAILED");
        return -1;
    }
    LOG_INF("STARTUP SUCCESS");
    return 0;
}