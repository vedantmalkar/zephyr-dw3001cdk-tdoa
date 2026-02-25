#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define ROLE 1 // 1 for tag & 0 for anchor
#define ANTENNA_DELAY 26200 
#define SPEED_OF_LIGHT 299702547.0
#define UUS_TO_DWT_TIME 63898// number of ticks per microsecond


#define MSG_POLL  0x01 //first messege sent (tag)
#define MSG_RESP  0x02  // messege send back (anchor)
#define MSG_FINAL 0x03  // final messege sent (tag)

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

static int uwb_init()
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

    dwt_setrxantennadelay(ANTENNA_DELAY);
    dwt_settxantennadelay(ANTENNA_DELAY);

    return 0;
}

#if ROLE

static void tag_loop(){
    uwb_init();
    uint8_t poll_msg[2] = {MSG_POLL,0};
    uint8_t sr = 0; 
    uint8_t t1[5];
    uint8_t t4[5];
    uint8_t t5[5];
    uint64_t t2 = 0, t6 = 0;
    uint8_t rx_buffer[32];

    while(1){
        poll_msg[1] = sr;
        dwt_writetxdata(sizeof(poll_msg), poll_msg,0);
        dwt_writetxfctrl(sizeof(poll_msg),0,0); 
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        LOG_INF("POLL Value Send %d", sr);
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));
        dwt_readtxtimestamp(t1);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //tx flag is cleared
        sr++;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG_BIT_MASK));
        dwt_readrxdata(rx_buffer, sizeof(rx_buffer),0 );
        if(rx_buffer[0] == MSG_RESP){
            dwt_readrxtimestamp(t5);
            for(int i = 0; i < 5; i++){
                t2 |= ((uint64_t)rx_buffer[1 + i]) << (8 * i);
                t6 |= ((uint64_t)rx_buffer[6 + i]) << (8 * i);
            }
        }else{
            LOG_ERR("NO RESP MSG");
            continue;
        }
            
            
    }

}

#else

#endif
