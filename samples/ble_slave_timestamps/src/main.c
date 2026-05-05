/*
 * BLE Slave Timestamp Streamer 
 * Pair this with: flash samples/wireless_time_sync_master on a second board.
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

LOG_MODULE_REGISTER(ble_slave, LOG_LEVEL_INF);

#define NODE_ID   2
#define ANT_DLY   26194
#define MSG_SYNC  0x10

#define MASK40  0xFFFFFFFFFFULL
#define HALF40  (1LL << 39)
#define FULL40  (1LL << 40)

struct slave_entry {
	uint8_t  seq;
	uint64_t tx_ts;
	uint64_t rx_ts;
	int64_t  offset;
	double   drift;
	double   corrected;
};

K_MSGQ_DEFINE(slave_queue, sizeof(struct slave_entry), 32, 4);

#define BT_UUID_NUS_VAL \
	BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9EULL)
#define BT_UUID_NUS_TX_VAL \
	BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9EULL)

static struct bt_uuid_128 nus_uuid    = BT_UUID_INIT_128(BT_UUID_NUS_VAL);
static struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_TX_VAL);

static struct bt_conn *current_conn;
static bool notify_enabled;

static void tx_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notifications %s", notify_enabled ? "enabled" : "disabled");
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

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_WRN("MTU exchange failed (err %u)", err);
	} else {
		LOG_INF("MTU exchanged: %u", bt_gatt_get_mtu(conn));
	}
}

static struct bt_gatt_exchange_params mtu_params = {
	.func = mtu_exchange_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connect failed (err %u)", err);
		return;
	}
	current_conn = bt_conn_ref(conn);
	LOG_INF("Connected");
	bt_gatt_exchange_mtu(conn, &mtu_params);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);
	bt_conn_unref(current_conn);
	current_conn = NULL;
	notify_enabled = false;
	bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected    = connected,
	.disconnected = disconnected,
};

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
		LOG_ERR("dwt_probe failed");
		return -1;
	}

	port_set_dw_ic_spi_fastrate();

	if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
		LOG_ERR("dwt_initialise failed");
		return -1;
	}

	if (dwt_configure(&uwb_cfg) != DWT_SUCCESS) {
		LOG_ERR("dwt_configure failed");
		return -1;
	}

	dwt_settxantennadelay(ANT_DLY);
	dwt_setrxantennadelay(ANT_DLY);

	return 0;
}

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

#define UWB_STACK_SIZE 4096
#define UWB_PRIORITY   5

static void uwb_rx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	LOG_INF("UWB RX thread started");

	uint8_t  rx_buf[32];
	uint64_t prev_tx = 0;
	uint64_t prev_rx = 0;
	double   drift   = 1.0;

	while (1) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		uint32_t status;

		while (!((status = dwt_readsysstatuslo()) &
			 (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
		}

		if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
			dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
			continue;
		}

		uint16_t len = dwt_getframelength();

		dwt_readrxdata(rx_buf, len - FCS_LEN, 0);

		if (rx_buf[0] != MSG_SYNC) {
			dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR);
			continue;
		}

		uint8_t  seq     = rx_buf[1];
		uint64_t rx_time = get_rx_ts();

		uint64_t tx_time = 0;

		for (int i = 0; i < 5; i++) {
			tx_time |= ((uint64_t)rx_buf[3 + i]) << (8 * i);
		}

		int64_t diff = (int64_t)((rx_time - tx_time) & MASK40);

		if (diff > HALF40) {
			diff -= FULL40;
		}

		int64_t offset = diff;

		if (prev_tx != 0) {
			uint64_t master_dt = tx_time - prev_tx;
			uint64_t slave_dt  = rx_time - prev_rx;

			drift = (double)master_dt / (double)slave_dt;
		}

		double corrected = ((double)rx_time - (double)offset) / drift;

		struct slave_entry entry = {
			.seq       = seq,
			.tx_ts     = tx_time,
			.rx_ts     = rx_time,
			.offset    = offset,
			.drift     = drift,
			.corrected = corrected,
		};

		if (k_msgq_put(&slave_queue, &entry, K_NO_WAIT) != 0) {
			k_msgq_purge(&slave_queue);
			k_msgq_put(&slave_queue, &entry, K_NO_WAIT);
		}

		prev_tx = tx_time;
		prev_rx = rx_time;

		dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR);
	}
}

K_THREAD_STACK_DEFINE(uwb_stack, UWB_STACK_SIZE);
static struct k_thread uwb_thread_data;

int main(void)
{
	LOG_INF("BLE Slave Timestamp Streamer starting");

	if (uwb_init() != 0) {
		LOG_ERR("DW3000 init failed");
		return -1;
	}
	LOG_INF("DW3000 ready");

	k_thread_create(&uwb_thread_data, uwb_stack, UWB_STACK_SIZE,
			uwb_rx_thread, NULL, NULL, NULL,
			UWB_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&uwb_thread_data, "uwb_rx");

	if (bt_enable(NULL) != 0) {
		LOG_ERR("BLE enable failed");
		return -1;
	}
	LOG_INF("BLE ready");

	if (bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd)) != 0) {
		LOG_ERR("Advertising start failed");
		return -1;
	}
	LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);

	struct slave_entry entry;
	char buf[80];

	while (1) {
		k_msgq_get(&slave_queue, &entry, K_FOREVER);

		int len = snprintf(buf, sizeof(buf),
				   "%u,%llu,%llu,%lld,%.9f,%.0f\n",
				   entry.seq,
				   entry.tx_ts,
				   entry.rx_ts,
				   entry.offset,
				   entry.drift,
				   entry.corrected);

		printk("%s", buf);

		if (notify_enabled && current_conn) {
			bt_gatt_notify(current_conn, TX_ATTR, buf, len);
		}
	}

	return 0;
}
