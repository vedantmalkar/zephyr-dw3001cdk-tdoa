/* TDOA TAG (BLINK TRANSMITTER) */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(tdoa_tag, LOG_LEVEL_INF);

#define TAG_ID 1
#define ANT_DLY 26194

#define MSG_BLINK 0x20

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

    return 0;
}

/* BLINK LOOP */

static void tag_loop(void)
{
    uint8_t tx_buf[2];
    static uint8_t blink_seq = 0;

    while(1)
    {
        tx_buf[0] = MSG_BLINK;
        tx_buf[1] = blink_seq++;   // ✅ sequence number

        dwt_writetxdata(2, tx_buf, 0);
        dwt_writetxfctrl(2 + FCS_LEN, 0, 0);

        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while(!(dwt_readsysstatuslo() &
               DWT_INT_TXFRS_BIT_MASK));

        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        LOG_INF("BLINK sent seq=%d", tx_buf[1]);

        k_msleep(100);  // 10 Hz
    }
}

int main(void)
{
    LOG_INF("TDOA Tag Start");

    if(uwb_init()!=0)
    {
        LOG_ERR("Init failed");
        return -1;
    }

    tag_loop();

    return 0;
}