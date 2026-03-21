/*
 * BLE TX Timestamp Streamer — DWM3001CDK
 *
 * Transmits a UWB blink every 100 ms, reads the 40-bit hardware TX timestamp,
 * prints it on the serial console, and sends it to a connected laptop over BLE
 * (Nordic UART Service — TX notify characteristic).
 *
 * Format sent over BLE and serial:
 *   "seq=<n>  ts=<decimal>\n"
 *
 * No second UWB device needed — the chip transmits to itself.
 *
 * Architecture:
 *   uwb_tx_thread  — transmit blink → wait TXFRS → read TX timestamp → msgq
 *   main thread    — BLE init + advertising + dequeue msgq → notify + printk
 *
 * Build:
 *   cd samples/ble_tx_timestamps
 *   west build -b decawave_dwm3001cdk
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

LOG_MODULE_REGISTER(ble_tx_ts, LOG_LEVEL_INF);

/* --------------------------------------------------------------------------
 * Message queue — UWB TX thread → main/BLE thread
 * -------------------------------------------------------------------------- */

struct ts_entry {
	uint8_t  seq;
	uint64_t tx_ts;
};

K_MSGQ_DEFINE(ts_queue, sizeof(struct ts_entry), 32, 4);

/* --------------------------------------------------------------------------
 * NUS UUIDs
 * -------------------------------------------------------------------------- */

#define BT_UUID_NUS_VAL \
	BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9EULL)
#define BT_UUID_NUS_TX_VAL \
	BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9EULL)

static struct bt_uuid_128 nus_uuid    = BT_UUID_INIT_128(BT_UUID_NUS_VAL);
static struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(BT_UUID_NUS_TX_VAL);

/* --------------------------------------------------------------------------
 * BLE state
 * -------------------------------------------------------------------------- */

static struct bt_conn *current_conn;
static bool notify_enabled;

/* --------------------------------------------------------------------------
 * GATT
 * -------------------------------------------------------------------------- */

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

/* --------------------------------------------------------------------------
 * Advertising
 * -------------------------------------------------------------------------- */

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

/* --------------------------------------------------------------------------
 * MTU exchange
 * -------------------------------------------------------------------------- */

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

/* --------------------------------------------------------------------------
 * Connection callbacks
 * -------------------------------------------------------------------------- */

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

/* --------------------------------------------------------------------------
 * DW3000 config and init
 * -------------------------------------------------------------------------- */

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

	dwt_setrxantennadelay(16385);
	dwt_settxantennadelay(16385);

	return 0;
}

/* --------------------------------------------------------------------------
 * UWB TX thread — transmit blink every 100ms, read TX timestamp, enqueue
 * -------------------------------------------------------------------------- */

#define UWB_STACK_SIZE 4096
#define UWB_PRIORITY   5

static void uwb_tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	LOG_INF("UWB TX thread started");

	uint8_t msg[] = {0xAB, 0xCD, 0x00};
	uint8_t ts_raw[5];
	uint8_t seq = 0;

	while (1) {
		msg[2] = seq;

		dwt_writetxdata(sizeof(msg), msg, 0);
		dwt_writetxfctrl(sizeof(msg) + FCS_LEN, 0, 0);
		dwt_starttx(DWT_START_TX_IMMEDIATE);

		/* Wait for TX done */
		while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) {
		}

		dwt_readtxtimestamp(ts_raw);
		dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

		uint64_t ts = 0;

		for (int i = 4; i >= 0; i--) {
			ts = (ts << 8) | ts_raw[i];
		}

		struct ts_entry entry = {
			.seq   = seq,
			.tx_ts = ts,
		};

		if (k_msgq_put(&ts_queue, &entry, K_NO_WAIT) != 0) {
			k_msgq_purge(&ts_queue);
			k_msgq_put(&ts_queue, &entry, K_NO_WAIT);
		}

		seq++;
		k_msleep(100);
	}
}

K_THREAD_STACK_DEFINE(uwb_stack, UWB_STACK_SIZE);
static struct k_thread uwb_thread_data;

/* --------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------- */

int main(void)
{
	LOG_INF("BLE TX Timestamp Streamer starting");

	if (uwb_init() != 0) {
		LOG_ERR("DW3000 init failed");
		return -1;
	}
	LOG_INF("DW3000 ready");

	/* Start UWB TX thread only after full DW3000 init */
	k_thread_create(&uwb_thread_data, uwb_stack, UWB_STACK_SIZE,
			uwb_tx_thread, NULL, NULL, NULL,
			UWB_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&uwb_thread_data, "uwb_tx");

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

	struct ts_entry entry;
	char buf[32];

	while (1) {
		/* Block until UWB thread enqueues a timestamp */
		k_msgq_get(&ts_queue, &entry, K_FOREVER);

		int len = snprintf(buf, sizeof(buf), "seq=%u ts=%llu\n",
				   entry.seq, entry.tx_ts);

		/* Always print to serial */
		printk("%s", buf);

		/* Send over BLE only if laptop is connected and subscribed */
		if (notify_enabled && current_conn) {
			bt_gatt_notify(current_conn, TX_ATTR, buf, len);
		}
	}

	return 0;
}
