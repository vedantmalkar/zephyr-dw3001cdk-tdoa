/*
 * BLE TDoA Slave — DWM3001CDK (CORRECTED)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_hw.h"
#include "port.h"

LOG_MODULE_REGISTER(ble_tdoa_slave, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */

#define NODE_ID   0
#define ANT_DLY   26194
#define MSG_SYNC  0x10
#define MSG_BLINK 0x20

#define MASK40  0xFFFFFFFFFFULL
#define HALF40  (1LL << 39)
#define FULL40  (1LL << 40)

/* -------------------------------------------------------------------------- */

struct tdoa_entry {
    uint8_t  id;
    uint8_t  type;
    uint8_t  seq;
    uint8_t  sync_seq;
    uint64_t rx_ts;
    uint64_t tx_ts;
    int64_t  offset;
    double   drift;
    double   corrected;
};

K_MSGQ_DEFINE(tdoa_queue, sizeof(struct tdoa_entry), 32, 4);

/* -------------------------------------------------------------------------- */
/* BLE UUIDs */

#define BT_UUID_NUS_VAL \
    BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9EULL)
#define BT_UUID_NUS_TX_VAL \
    BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9EULL)

static struct bt_uuid_128 nus_uuid    = BT_UUID_INIT_128(BT_UUID_NUS_VAL);
static struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_TX_VAL);

/* -------------------------------------------------------------------------- */

static struct bt_conn *current_conn;
static bool notify_enabled;

/* -------------------------------------------------------------------------- */

static void tx_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(nus_svc,
    BT_GATT_PRIMARY_SERVICE(&nus_uuid),
    BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, NULL),
    BT_GATT_CCC(tx_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

#define TX_ATTR (&nus_svc.attrs[1])

/* -------------------------------------------------------------------------- */

static const struct bt_le_adv_param adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN,
        BT_GAP_ADV_FAST_INT_MIN_2,
        BT_GAP_ADV_FAST_INT_MAX_2,
        NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
        sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* -------------------------------------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) return;
    current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    bt_conn_unref(current_conn);
    current_conn = NULL;
    notify_enabled = false;
    bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* -------------------------------------------------------------------------- */

static dwt_config_t uwb_cfg = {
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

/* -------------------------------------------------------------------------- */

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

    if (dwt_configure(&uwb_cfg) != DWT_SUCCESS)
        return -1;

    dwt_settxantennadelay(ANT_DLY);
    dwt_setrxantennadelay(ANT_DLY);

    return 0;
}

/* -------------------------------------------------------------------------- */

static uint64_t get_rx_ts(void)
{
    uint8_t ts[5];
    dwt_readrxtimestamp(ts);

    uint64_t val = 0;
    for (int i = 4; i >= 0; i--) {
        val = (val << 8) | ts[i];
    }
    return val;
}

/* -------------------------------------------------------------------------- */

#define UWB_STACK_SIZE 4096
#define UWB_PRIORITY   5

static void uwb_rx_thread(void *a, void *b, void *c)
{
    uint8_t  rx_buf[32];
    uint64_t prev_tx = 0;
    uint64_t prev_rx = 0;
    uint8_t  prev_seq = 0;
    int64_t  offset  = 0;
    double   drift   = 1.0;

    while (1) {

        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status;
        while (!((status = dwt_readsysstatuslo()) &
            (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {}

        if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
            continue;
        }

        uint16_t len = dwt_getframelength();
        dwt_readrxdata(rx_buf, len - FCS_LEN, 0);

        uint64_t rx_time = get_rx_ts();

        /* ---------- BLINK ---------- */

        if (rx_buf[0] == MSG_BLINK) {

            struct tdoa_entry entry = {
                .id        = NODE_ID,
                .type      = MSG_BLINK,
                .seq       = rx_buf[1],
                .sync_seq  = 0,
                .rx_ts     = rx_time,
                .tx_ts     = 0,
                .offset    = 0,
                .drift     = 1.0,
                .corrected = 0.0,
            };

            if (prev_tx != 0) {
                uint64_t slave_dt = (rx_time - prev_rx) & MASK40;
                entry.sync_seq = prev_seq;
                entry.tx_ts = prev_tx;
                entry.corrected =
                    (double)prev_tx + (double)slave_dt * drift;

                k_msgq_put(&tdoa_queue, &entry, K_NO_WAIT);
            }
        }

        /* ---------- SYNC ---------- */

        if (rx_buf[0] == MSG_SYNC) {

            uint8_t seq = rx_buf[1];

            uint64_t tx_time = 0;
            for (int i = 0; i < 5; i++) {
                tx_time |= ((uint64_t)rx_buf[3 + i]) << (8 * i);
            }

            int64_t diff = (int64_t)((rx_time - tx_time) & MASK40);
            if (diff > HALF40) diff -= FULL40;
            offset = diff;

            if (prev_tx != 0) {
                uint64_t master_dt = (tx_time - prev_tx) & MASK40;
                uint64_t slave_dt  = (rx_time - prev_rx) & MASK40;

                if (slave_dt != 0)
                    drift = (double)master_dt / (double)slave_dt;
            }

            double corrected = (prev_tx != 0)
                ? (double)prev_tx + ((double)rx_time - (double)prev_rx) * drift
                : (double)tx_time;

            struct tdoa_entry entry = {
                .id        = NODE_ID,
                .type      = MSG_SYNC,
                .seq       = seq,
                .sync_seq  = seq,
                .rx_ts     = rx_time,
                .tx_ts     = tx_time,
                .offset    = offset,
                .drift     = drift,
                .corrected = corrected,
            };

            k_msgq_put(&tdoa_queue, &entry, K_NO_WAIT);

            prev_seq = seq;
            prev_tx = tx_time;
            prev_rx = rx_time;
        }

        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR);
    }
}

/* -------------------------------------------------------------------------- */

K_THREAD_STACK_DEFINE(uwb_stack, UWB_STACK_SIZE);
static struct k_thread uwb_thread_data;

/* -------------------------------------------------------------------------- */

int main(void)
{
    if (uwb_init() != 0) return -1;

    k_thread_create(&uwb_thread_data, uwb_stack, UWB_STACK_SIZE,
        uwb_rx_thread, NULL, NULL, NULL,
        UWB_PRIORITY, 0, K_NO_WAIT);

    if (bt_enable(NULL) != 0) return -1;

    bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
                    sd, ARRAY_SIZE(sd));

    struct tdoa_entry entry;
    char buf[96];

    while (1) {

        k_msgq_get(&tdoa_queue, &entry, K_FOREVER);

        int len;

        if (entry.type == MSG_SYNC) {
            len = snprintf(buf, sizeof(buf),
                "SYNC,%u,%u,%llu,%llu,%lld,%.9f,%.0f\n",
                entry.id, entry.seq,
                entry.tx_ts, entry.rx_ts,
                entry.offset, entry.drift,
                entry.corrected);
        } else {
            len = snprintf(buf, sizeof(buf),
                "BLINK,%u,%u,%u,%llu,%.0f\n",
                entry.id, entry.seq,
                entry.sync_seq, entry.tx_ts,
                entry.corrected);
        }

        printk("%s", buf);

        if (notify_enabled && current_conn) {
            bt_gatt_notify(current_conn, TX_ATTR, buf, len);
        }
    }
}