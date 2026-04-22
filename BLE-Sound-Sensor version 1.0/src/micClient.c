#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>

#include "micsense_service.h"  // reuse UUIDs from your peripheral project


// Define db_int
int16_t db_int = 0;


static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

/* Notification handler */
static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    if (!data) {
        printk("Notification stopped\n");
        return BT_GATT_ITER_STOP;
    }

    printk("Notification received (%u bytes): ", length);
    for (int i = 0; i < length; i++) {
        printk("%02x ", ((uint8_t *)data)[i]);
    }
    printk("\n");

    return BT_GATT_ITER_CONTINUE;
}

/* GATT discovery callback */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("Discover complete\n");
        memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
    char uuid_str[BT_UUID_STR_LEN];
    bt_uuid_to_str(chrc->uuid, uuid_str, sizeof(uuid_str));

    printk("Discovered characteristic %s handle %u\n", uuid_str, chrc->value_handle);

    /* Subscribe to the Threshold Alert Characteristic */
    if (!bt_uuid_cmp(chrc->uuid,
                     BT_UUID_DECLARE_128(BT_UUID_THRESHOLD_ALERT_CHARACTERISTIC_VAL))) {
        printk("Subscribing to Threshold Alert notifications...\n");
        subscribe_params.notify = notify_func;
        subscribe_params.value = BT_GATT_CCC_NOTIFY;
        subscribe_params.ccc_handle = 0; // will be auto-filled
        subscribe_params.value_handle = chrc->value_handle;
        int err = bt_gatt_subscribe(conn, &subscribe_params);
        if (err) {
            printk("Subscribe failed (err %d)\n", err);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    printk("Connected\n");
    default_conn = bt_conn_ref(conn);

    /* Start service discovery */
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid = NULL; // discover all
    discover_params.func = discover_func;
    discover_params.start_handle = 0x0001;
    discover_params.end_handle = 0xffff;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int ret = bt_gatt_discover(default_conn, &discover_params);
    if (ret) {
        printk("Discover failed (err %d)\n", ret);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);

    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* Advertisement parsing */
static bool ad_parse_cb(struct bt_data *data, void *user_data)
{
    const bt_addr_le_t *addr = user_data;

    // Check if the data type is complete local name or shortened local name
    if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
        char name[BT_UUID_STR_LEN];
        memcpy(name, data->data, MIN(data->data_len, sizeof(name) - 1));
        name[data->data_len] = '\0'; // Null-terminate the string

        printk("Device name: %s\n", name);

        // Check if the name contains "Mic-Sense"
        if (strstr(name, "Mic-Sense")) {
            printk("Found Mic-Sense device: %s\n", name);

            // Stop scanning and connect to the device
            bt_le_scan_stop();

            struct bt_conn_le_create_param *create_param = BT_CONN_LE_CREATE_PARAM(
                BT_CONN_LE_CREATE_CONN,
                BT_GAP_SCAN_FAST_INTERVAL,
                BT_GAP_SCAN_FAST_WINDOW
            );

            struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;

            int err = bt_conn_le_create(addr, create_param, param, &default_conn);
            if (err) {
                printk("Create connection failed (err %d)\n", err);
            }

            return false; // Stop parsing further advertisement data
        }
    }

    return true; // Continue parsing other advertisement data
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi,
                         uint8_t type, struct net_buf_simple *ad)
{
    char dev[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, dev, sizeof(dev));
    printk("Device found: %s (RSSI: %d)\n", dev, rssi);

    // Parse advertisement data
    bt_data_parse(ad, ad_parse_cb, (void *)addr);
}

int main(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    printk("Central ready, starting scan\n");

    err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    if (err) {
        printk("Scanning failed to start (err %d)\n", err);
        return err;
    }

    return 0;
}
