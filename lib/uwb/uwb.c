#include "uwb.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

dwt_config_t uwb_default_config = {
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

int uwb_init(uint16_t ant_dly)
{
    dw_device_init();
    dw3000_hw_wakeup_pin_low();
    Sleep(5);

    port_set_dw_ic_spi_slowrate();

    if(dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS)
        return -1;

    port_set_dw_ic_spi_fastrate();

    if(dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS)
        return -1;

    if(dwt_configure(&uwb_default_config) != DWT_SUCCESS)
        return -1;

    if(ant_dly > 0)
    {
        dwt_setrxantennadelay(ant_dly);
        dwt_settxantennadelay(ant_dly);
    }

    return 0;
}

uint64_t uwb_get_tx_ts(void)
{
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);

    uint64_t val = 0;
    for(int i = 4; i >= 0; i--)
        val = (val << 8) | ts[i];

    return val;
}

uint64_t uwb_get_rx_ts(void)
{
    uint8_t ts[5];
    dwt_readrxtimestamp(ts);

    uint64_t val = 0;
    for(int i = 4; i >= 0; i--)
        val = (val << 8) | ts[i];

    return val;
}

uint64_t uwb_get_sys_time(void)
{
    uint8_t ts[5];
    dwt_readsystime(ts);

    uint64_t val = 0;
    for(int i = 4; i >= 0; i--)
        val = (val << 8) | ts[i];

    return val;
}

void uwb_tx(uint8_t *data, uint16_t len)
{
    dwt_writetxdata(len, data, 0);
    dwt_writetxfctrl(len + FCS_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    while(!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK));
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
}

int uwb_tx_delayed(uint8_t *data, uint16_t len, uint32_t tx_time)
{
    dwt_setdelayedtrxtime(tx_time);
    dwt_writetxdata(len, data, 0);
    dwt_writetxfctrl(len + FCS_LEN, 0, 0);

    if(dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS)
        return -1;

    while(!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK));
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    return 0;
}

int uwb_rx(uint8_t *rx_buf, uint16_t *rx_len)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t status;
    while(!((status = dwt_readsysstatuslo()) &
          (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    if(!(status & DWT_INT_RXFCG_BIT_MASK))
    {
        uwb_clear_status();
        return -1;
    }

    uint16_t len = dwt_getframelength();
    dwt_readrxdata(rx_buf, len - FCS_LEN, 0);

    if(rx_len)
        *rx_len = len - FCS_LEN;

    return 0;
}

void uwb_clear_status(void)
{
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK |
        SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR |
        DWT_INT_TXFRS_BIT_MASK);
}

void uwb_pack_ts(uint64_t ts, uint8_t *buf)
{
    for(int i = 0; i < 5; i++)
        buf[i] = (ts >> (8 * i));
}

uint64_t uwb_unpack_ts(const uint8_t *buf)
{
    uint64_t val = 0;
    for(int i = 0; i < 5; i++)
        val |= ((uint64_t)buf[i]) << (8 * i);

    return val;
}
