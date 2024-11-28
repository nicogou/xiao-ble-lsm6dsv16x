#include "app/lib/xiao_smp_bluetooth.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(smp_bt, CONFIG_XIAO_SMP_BLUETOOTH_LOG_LEVEL);

#define BT_UUID_SMP_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x8D53DC1D, 0x1DB7, 0x4CD3, 0x868B, 0x8A527460AA84)

static struct k_work advertise_work;

static xiao_smp_bluetooth_cb_t callbacks;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SMP_SERVICE_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void advertise(struct k_work *work)
{
	int rc;

	bt_le_adv_stop();

	rc = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (rc == -EALREADY)
	{
		LOG_WRN("Advertising already started!");
	} else if (rc) {
		LOG_ERR("Advertising failed to start (rc %d)", rc);
		return;
	}

	LOG_INF("Advertising successfully started");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)", err);
		if (callbacks.on_connection_fail)
		{
			(*callbacks.on_connection_fail)(err);
		}
	} else {
		LOG_INF("Connected");
		if (callbacks.on_connection_success)
		{
			(*callbacks.on_connection_success)();
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)", reason);
	if (callbacks.on_disconnection)
	{
		(*callbacks.on_disconnection)(reason);
	}
	k_work_submit(&advertise_work);
}

/*
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};
*/

static void bt_ready(int err)
{
	if (err != 0) {
		LOG_ERR("Bluetooth failed to initialise: %d", err);
	} else {
		k_work_submit(&advertise_work);
	}
}

void smp_bluetooth_init(xiao_smp_bluetooth_cb_t cb)
{
	int rc;

	callbacks = cb;

	k_work_init(&advertise_work, advertise);
	rc = bt_enable(bt_ready);

	if (rc != 0) {
		LOG_ERR("Bluetooth enable failed: %d", rc);
	}
}

int smp_bluetooth_start_advertising()
{
	return k_work_submit(&advertise_work);
}

int smp_bluetooth_stop_advertising()
{
	return bt_le_adv_stop();
}
