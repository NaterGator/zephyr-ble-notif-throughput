/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/shell/shell_uart.h>


#include "main.h"

#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define MTU_OVERHEAD 3

static ssize_t write_cmd_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            const void *buf,
                            uint16_t len,
                            uint16_t offset,
                            uint8_t flags);
static void notif_ccc_cb(const struct bt_gatt_attr *attr, uint16_t value);

static volatile bool data_length_req;
static volatile bool test_ready;
static volatile bool m_notif_enabled = false;
static volatile bool m_notif_send    = false;
static volatile uint16_t m_mtu = 23;
static struct bt_conn *default_conn;

static uint8_t  m_msg_buffer[CONFIG_BT_L2CAP_TX_MTU - MTU_OVERHEAD];
static uint32_t m_msg_idx_cnt = 0;

#define SERVICE_UUID_BYTES 0xf4, 0xec, 0x36, 0x41, 0xde, 0x4b, 0x45, 0xa7, \
                           0xf8, 0x4a, 0xbd, 0x54, 0x64, 0xe4, 0xb3, 0x1f
static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(SERVICE_UUID_BYTES);
static struct bt_uuid_16  cmd_uuid     = BT_UUID_INIT_16(0x1000);
static struct bt_uuid_16  notif_uuid   = BT_UUID_INIT_16(0x1001);


struct bt_gatt_attr m_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&service_uuid),
    BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&cmd_uuid,
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL,
                           write_cmd_cb,
                           NULL),
    BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&notif_uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL,
                           NULL,
                           NULL),
    BT_GATT_CCC(notif_ccc_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};
static struct bt_gatt_service m_svcs = BT_GATT_SERVICE(m_attrs);
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SERVICE_UUID_BYTES),
};

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

ssize_t static write_cmd_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            const void *buf,
                            uint16_t len,
                            uint16_t offset,
                            uint8_t flags)
{
	const uint8_t *dptr = (const uint8_t*)buf;
	if (len >= 2) {
		if (dptr[0] == 0x01) {
			// set notification streaming on buf[1]
			m_notif_send = dptr[1] == 0x01;
		}
	}
	return len;
}

static void notif_ccc_cb(const struct bt_gatt_attr *attr, uint16_t value) 
{
	m_notif_enabled = (value & BT_GATT_CCC_NOTIFY) ? true : false;
}

// Write to the notification characteristic fragmenting at MTU as quickly as possible
static int send_data(void *data, const uint16_t len, struct bt_gatt_attr *attr)
{
	if (default_conn == NULL) {
		return -ENODEV;
	}
	int err = 0;

	const uint16_t mtu = m_mtu;
	if (len <= (mtu - MTU_OVERHEAD)) {
		err = bt_gatt_notify(default_conn, attr, data, len);
	} else {
		const uint8_t *frag = (const uint8_t *)data;
		uint16_t frag_len = mtu- MTU_OVERHEAD;
		uint16_t remaining_len = len - frag_len;
		err = bt_gatt_notify(default_conn, attr, frag, frag_len);
		while(!err && remaining_len > 0) {
			frag += frag_len;
			if (remaining_len <= (mtu - MTU_OVERHEAD)) {
				frag_len = remaining_len;
				remaining_len = 0;
			} else {
				remaining_len -= frag_len;
			}
			
			err = bt_gatt_notify(default_conn, attr, frag, frag_len);
		} 
	}
	return err;
}

static void connected(struct bt_conn *conn, uint8_t hci_err)
{
	struct bt_conn_info info = {0};
	int err;

	if (hci_err) {
		if (hci_err == BT_HCI_ERR_UNKNOWN_CONN_ID) {
			/* Canceled creating connection */
			return;
		}

		printk("Connection failed (err 0x%02x)\n", hci_err);
		return;
	}

	if (default_conn) {
		printk("Connection exists, disconnect second connection\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	default_conn = bt_conn_ref(conn);

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		printk("Failed to get connection info %d\n", err);
		return;
	}
	m_mtu = 23;
	printk("Conn. interval is %u units\n", info.le.interval);

}


static void adv_start(void)
{
	struct bt_le_adv_param *adv_param =
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE |
		                BT_LE_ADV_OPT_ONE_TIME,
		                BT_GAP_ADV_FAST_INT_MIN_2,
		                BT_GAP_ADV_FAST_INT_MAX_2,
		                NULL);
	int err;

	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd,
	                      ARRAY_SIZE(sd));
	if (err) {
		printk("Failed to start advertiser (%d)\n", err);
		return;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info info = {0};
	int err;

	printk("Disconnected (reason 0x%02x)\n", reason);

	test_ready = false;
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (%d)\n", err);
		return;
	}

	/* Re-connect using same roles */
	adv_start();
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	printk("Connection parameters update request received.\n");
	printk("Minimum interval: %d, Maximum interval: %d\n",
	       param->interval_min, param->interval_max);
	printk("Latency: %d, Timeout: %d\n", param->latency, param->timeout);

	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout)
{
	printk("Connection parameters updated.\n"
	       " interval: %d, latency: %d, timeout: %d\n",
	       interval, latency, timeout);
}

static void le_phy_updated(struct bt_conn *conn,
                           struct bt_conn_le_phy_info *param)
{
	printk("LE PHY updated: TX PHY %s, RX PHY %s\n",
	       phy2str(param->tx_phy), phy2str(param->rx_phy));

}

static void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{

	printk("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)\n", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_length_updated
};

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
	m_mtu = tx;
	if (m_mtu >= CONFIG_BT_L2CAP_TX_MTU) {
		m_mtu = CONFIG_BT_L2CAP_TX_MTU;
	}
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated,
};


// Thread to pump data out the notification as quickly as possible
static void notify_thread(void *, void *, void *)
{
	// Message pump for the notification characteristic
	while(1) {
		if (m_notif_enabled && m_notif_send) {
			// Ensure each notification fits nicely without fragmenting.
			const size_t len = m_mtu - MTU_OVERHEAD;
			for (int i = 0; i < len; i++) {
				const uint8_t shift = (m_msg_idx_cnt & 1) ? 9 : 1;
				m_msg_buffer[i] = (m_msg_idx_cnt++ >> shift) & 0xFF;
			}
			if (m_msg_idx_cnt > (UINT16_MAX << 1)) {
				m_msg_idx_cnt %= (UINT16_MAX << 1);
			}
			send_data(m_msg_buffer, len, &m_attrs[3]);
		} else {
			k_msleep(100);
		}
	}
}

#define NOTIFY_THREAD_STACKSIZE 2048
#define NOTIFY_THREAD_PRIORITY  8
K_THREAD_DEFINE(notify_thread_id, NOTIFY_THREAD_STACKSIZE, notify_thread,
                NULL, NULL, NULL, // unused args
                NOTIFY_THREAD_PRIORITY, 0, 0);


int main(void)
{
	int err;

	printk("Starting Bluetooth Throughput example v1.0.1\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
	
	printk("\nStarting advertising\n");
	bt_gatt_cb_register(&gatt_callbacks);
	bt_gatt_service_register(&m_svcs);
	adv_start();
	return 0;
}
