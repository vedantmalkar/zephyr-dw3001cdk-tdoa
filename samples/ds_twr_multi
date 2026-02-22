#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(ds_twr_multi, LOG_LEVEL_INF);

/* ================= USER CONFIG ================= */

#define ROLE_INITIATOR 1   // 1 = Tag, 0 = Anchor
#define NODE_ID        0   // Tag = 0, Anchors = 1,2,3,...

#define ANT_DLY 26194     // <-- Use your calibrated value
#define SPEED_OF_LIGHT 299702547.0
#define UUS_TO_DWT_TIME 63898

#define POLL_RX_TO_RESP_TX_DLY_UUS   900
#define RESP_RX_TO_FINAL_TX_DLY_UUS  900

#define MSG_POLL  0x01
#define MSG_RESP  0x02
#define MSG_FINAL 0x03

/* =============================================== */

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

/* ================= TIMESTAMP HELPERS ================= */

static uint64_t get_tx_ts(void)
{
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);
    uint64_t val = 0;
    for(int i=4;i>=0;i--)
        val = (val<<8) | ts[i];
    return val;
}

static uint64_t get_rx_ts(void)
{
    uint8_t ts[5];
    dwt_readrxtimestamp(ts);
    uint64_t val = 0;
    for(int i=4;i>=0;i--)
        val = (val<<8) | ts[i];
    return val;
}

/* ================= UWB INIT ================= */

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
    dwt_setrxantennadelay(ANT_DLY);

    return 0;
}

/* ===================================================== */
/* ==================== INITIATOR ======================= */
/* ===================================================== */

#if ROLE_INITIATOR

#define NUM_ANCHORS 3
static uint8_t anchor_list[NUM_ANCHORS] = {1,2,3};

static void initiator_loop(void)
{
    uint8_t poll_msg[4];
    uint8_t resp_msg[32];
    uint8_t final_msg[32];

    uint8_t seq = 0;

    while(1)
    {
        for(int a=0; a<NUM_ANCHORS; a++)
        {
            uint8_t anchor_id = anchor_list[a];

            /* --------- SEND POLL ---------- */

            poll_msg[0] = MSG_POLL;
            poll_msg[1] = seq;
            poll_msg[2] = anchor_id;   // DEST
            poll_msg[3] = NODE_ID;     // SRC

            dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
            dwt_writetxfctrl(sizeof(poll_msg)+FCS_LEN,0,0);
            dwt_starttx(DWT_START_TX_IMMEDIATE);

            while(!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK));
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

            uint64_t t1 = get_tx_ts();

            /* --------- WAIT RESP ---------- */

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            uint32_t status;

            while(!((status=dwt_readsysstatuslo()) &
                  (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

            if(!(status & DWT_INT_RXFCG_BIT_MASK))
                continue;

            uint16_t len = dwt_getframelength();
            dwt_readrxdata(resp_msg, len-FCS_LEN, 0);

            if(resp_msg[0] != MSG_RESP ||
               resp_msg[2] != NODE_ID)
                continue;

            uint64_t t4 = get_rx_ts();

            uint64_t t2 = 0, t3 = 0;

            for(int i=0;i<5;i++)
            {
                t2 |= ((uint64_t)resp_msg[4+i])<<(8*i);
                t3 |= ((uint64_t)resp_msg[9+i])<<(8*i);
            }

            /* --------- SEND FINAL ---------- */

            uint32_t final_tx_time =
            (t4 + RESP_RX_TO_FINAL_TX_DLY_UUS *
            UUS_TO_DWT_TIME) >> 8;

            dwt_setdelayedtrxtime(final_tx_time);

            uint64_t t5 =
            (((uint64_t)(final_tx_time & 0xFFFFFFFE))<<8);

            final_msg[0] = MSG_FINAL;
            final_msg[1] = seq;
            final_msg[2] = anchor_id;
            final_msg[3] = NODE_ID;

            for(int i=0;i<5;i++)
                final_msg[4+i]=(t1>>(8*i));

            for(int i=0;i<5;i++)
                final_msg[9+i]=(t4>>(8*i));

            for(int i=0;i<5;i++)
                final_msg[14+i]=(t5>>(8*i));

            dwt_writetxdata(19, final_msg, 0);
            dwt_writetxfctrl(19+FCS_LEN,0,0);
            dwt_starttx(DWT_START_TX_DELAYED);

            while(!(dwt_readsysstatuslo() &
                   DWT_INT_TXFRS_BIT_MASK));
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

            /* --------- CALCULATE DIST ---------- */

            double Ra = (double)(t4 - t1);
            double Rb = (double)(t5 - t3);
            double Da = (double)(t5 - t4);
            double Db = (double)(t3 - t2);

            double tof =
            (Ra*Rb - Da*Db)/(Ra+Rb+Da+Db);

            double distance =
            tof * DWT_TIME_UNITS *
            SPEED_OF_LIGHT;

            LOG_INF("Anchor %d : %.2f m",
                    anchor_id,
                    distance);

            Sleep(50);
        }

        seq++;
        Sleep(200);
    }
}

#else

/* ===================================================== */
/* ====================== ANCHOR ======================== */
/* ===================================================== */

static void responder_loop(void)
{
    uint8_t rx_buf[32];
    uint8_t resp_msg[32];

    while(1)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;

        while(!((status=dwt_readsysstatuslo()) &
              (DWT_INT_RXFCG_BIT_MASK |
               SYS_STATUS_ALL_RX_ERR)));

        if(!(status & DWT_INT_RXFCG_BIT_MASK))
            continue;

        uint16_t len=dwt_getframelength();
        dwt_readrxdata(rx_buf,len-FCS_LEN,0);

        if(rx_buf[0]==MSG_POLL &&
           rx_buf[2]==NODE_ID)
        {
            uint8_t seq = rx_buf[1];
            uint8_t tag_id = rx_buf[3];

            uint64_t t2 = get_rx_ts();

            uint32_t resp_tx_time =
            (t2 + POLL_RX_TO_RESP_TX_DLY_UUS *
             UUS_TO_DWT_TIME) >> 8;

            dwt_setdelayedtrxtime(resp_tx_time);

            uint64_t t3 =
            (((uint64_t)(resp_tx_time &
              0xFFFFFFFE))<<8);

            resp_msg[0]=MSG_RESP;
            resp_msg[1]=seq;
            resp_msg[2]=tag_id;
            resp_msg[3]=NODE_ID;

            for(int i=0;i<5;i++)
                resp_msg[4+i]=(t2>>(8*i));

            for(int i=0;i<5;i++)
                resp_msg[9+i]=(t3>>(8*i));

            dwt_writetxdata(14,resp_msg,0);
            dwt_writetxfctrl(14+FCS_LEN,0,0);
            dwt_starttx(DWT_START_TX_DELAYED);

            while(!(dwt_readsysstatuslo() &
                   DWT_INT_TXFRS_BIT_MASK));
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        }

        dwt_writesysstatuslo(
        DWT_INT_RXFCG_BIT_MASK |
        SYS_STATUS_ALL_RX_ERR);
    }
}

#endif

/* ==================== MAIN ==================== */

int main(void)
{
    LOG_INF("Multi-Anchor DS-TWR Start");

    if(uwb_init()!=0)
    {
        LOG_ERR("Init failed");
        return -1;
    }

#if ROLE_INITIATOR
    LOG_INF("Initiator (Tag) ID=%d", NODE_ID);
    initiator_loop();
#else
    LOG_INF("Anchor ID=%d", NODE_ID);
    responder_loop();
#endif

    return 0;
}