#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <soc.h>
#include <stdio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include "micsense_service.h"

/* UUIDs Definitions */
#define BT_UUID_GET_THRESHOLD_SERVICE           BT_UUID_DECLARE_128(BT_UUID_GET_THRESHOLD_SERVICE_VAL)
#define BT_UUID_GET_THRESHOLD_CHARACTERISTIC    BT_UUID_DECLARE_128(BT_UUID_GET_THRESHOLD_CHARACTERISTIC_VAL)

#define BT_UUID_SET_THRESHOLD_SERVICE           BT_UUID_DECLARE_128(BT_UUID_SET_THRESHOLD_SERVICE_VAL)
#define BT_UUID_SET_THRESHOLD_CHARACTERISTIC    BT_UUID_DECLARE_128(BT_UUID_SET_THRESHOLD_CHARACTERISTIC_VAL)

#define BT_UUID_GET_BATTERY_SERVICE              BT_UUID_DECLARE_128(BT_UUID_GET_BATTERY_SERVICE_VAL)
#define BT_UUID_GET_BATTERY_CHARACTERISTIC       BT_UUID_DECLARE_128(BT_UUID_GET_BATTERY_CHARACTERISTIC_VAL)

#define BT_UUID_GET_SOUND_LEVEL_SERVICE          BT_UUID_DECLARE_128(BT_UUID_GET_SOUND_LEVEL_SERVICE_VAL)
#define BT_UUID_GET_SOUND_LEVEL_CHARACTERISTIC   BT_UUID_DECLARE_128(BT_UUID_GET_SOUND_LEVEL_CHARACTERISTIC_VAL)

#define BT_UUID_SET_SOUND_LEVEL_SERVICE          BT_UUID_DECLARE_128(BT_UUID_SET_SOUND_LEVEL_SERVICE_VAL)
#define BT_UUID_SET_SOUND_LEVEL_CHARACTERISTIC   BT_UUID_DECLARE_128(BT_UUID_SET_SOUND_LEVEL_CHARACTERISTIC_VAL)

#define BT_UUID_THRESHOLD_ALERT_SERVICE          BT_UUID_DECLARE_128(BT_UUID_THRESHOLD_ALERT_SERVICE_VAL)
#define BT_UUID_THRESHOLD_ALERT_CHARACTERISTIC   BT_UUID_DECLARE_128(BT_UUID_THRESHOLD_ALERT_CHARACTERISTIC_VAL)

#define BT_UUID_CALIBRATE_SERVICE                BT_UUID_DECLARE_128(BT_UUID_CALIBRATE_SERVICE_VAL)
#define BT_UUID_CALIBRATE_CHARACTERISTIC         BT_UUID_DECLARE_128(BT_UUID_CALIBRATE_CHARACTERISTIC_VAL)

#define BT_UUID_GET_NOISE_FLOOR_SERVICE           BT_UUID_DECLARE_128(BT_UUID_GET_NOISE_FLOOR_SERVICE_VAL)
#define BT_UUID_GET_NOISE_FLOOR_CHARACTERISTIC    BT_UUID_DECLARE_128(BT_UUID_GET_NOISE_FLOOR_CHARACTERISTIC_VAL)

#define BT_UUID_MICSENESE BT_UUID_DECLARE_128(BT_UUID_MICSENESE_VAL)
#define MAX_TRANSMIT_SIZE 1024

volatile bool ble_ready = false;

struct bt_conn *my_connection = NULL;
bool notify_enabled = false;

uint8_t threshold_value         = 50;
static uint8_t battery_level    = 40;
static uint8_t sound_level      = 0;
uint8_t sound_streaming_enabled = 0;
volatile uint8_t calibration_requested = 0;
uint8_t noise_floor_db = 0;
static bool threshold_alert_enabled = false;
uint8_t alertThreshold;

int MICSENSE_service_init(void)
{
    return 0;
}

static ssize_t on_receive(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf,
                          uint16_t len,
                          uint16_t offset,
                          uint8_t flags)
{
    uint8_t received_value;
    memcpy(&received_value, buf, sizeof(uint8_t));
    threshold_value = received_value;
    printk("Received integer: %d\n", received_value);
    printk("------Value stored in threshold variable: %d\n", threshold_value);
    return len;
}

static ssize_t on_SetStream(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags)
{
    uint8_t received_value;
    memcpy(&received_value, buf, sizeof(uint8_t));
    sound_streaming_enabled = received_value;
    printk("Received Set Stream: %d\n", received_value);
    return len;
}

ssize_t on_getThreshold(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset)
{
    printk("In On get Threshold function\n");
    return bt_gatt_attr_read(conn, attr, (void *)buf, len, offset,
                             &threshold_value, sizeof(threshold_value));
}

void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch (value) {
    case BT_GATT_CCC_NOTIFY:
        printk("Notifications enabled\n");
        notify_enabled = true;
        break;
    case BT_GATT_CCC_INDICATE:
        break;
    case 0:
        printk("Notifications disabled\n");
        notify_enabled = false;
        break;
    default:
        printk("Error, CCCD has been set to an invalid value\n");
    }
}

static ssize_t read_hello(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    void *buf,
    uint16_t len,
    uint16_t offset)
{
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(uint8_t));
}

BT_GATT_SERVICE_DEFINE(setThreshold,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SET_THRESHOLD_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_SET_THRESHOLD_CHARACTERISTIC,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, on_receive, NULL),
);

BT_GATT_SERVICE_DEFINE(setStreamService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SET_SOUND_LEVEL_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_SET_SOUND_LEVEL_CHARACTERISTIC,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, on_SetStream, NULL),
);

BT_GATT_SERVICE_DEFINE(getThreshold,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GET_THRESHOLD_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_GET_THRESHOLD_CHARACTERISTIC,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &threshold_value),
);

BT_GATT_SERVICE_DEFINE(batteryService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GET_BATTERY_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_GET_BATTERY_CHARACTERISTIC,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &battery_level),
);

const struct bt_gatt_attr *alert_threshold_attr;

BT_GATT_SERVICE_DEFINE(alertThresholdSrvc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_THRESHOLD_ALERT_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_THRESHOLD_ALERT_CHARACTERISTIC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &alertThreshold),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

const struct bt_gatt_attr *getStreamService_attr;

BT_GATT_SERVICE_DEFINE(getStreamService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GET_SOUND_LEVEL_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_GET_SOUND_LEVEL_CHARACTERISTIC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &db_int),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static ssize_t on_calibrate(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags)
{
    uint8_t received_value;
    memcpy(&received_value, buf, sizeof(uint8_t));
    if (received_value == 1) {
        calibration_requested = 1;
        printk("Calibration requested via BLE\n");
    }
    return len;
}

BT_GATT_SERVICE_DEFINE(calibrateService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CALIBRATE_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CALIBRATE_CHARACTERISTIC,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, on_calibrate, NULL),
);

BT_GATT_SERVICE_DEFINE(noiseFloorService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GET_NOISE_FLOOR_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_GET_NOISE_FLOOR_CHARACTERISTIC,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &noise_floor_db),
);

void setup_alert_service(void)
{
    alert_threshold_attr  = &alertThresholdSrvc.attrs[1];
    getStreamService_attr = &getStreamService.attrs[1];
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];
    my_connection = conn;

    if (err) {
        printf("Connection failed (err %u)\n", err);
        return;
    } else if (bt_conn_get_info(conn, &info)) {
        printf("Could not parse info\n");
    } else {
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        printf("Connection established! Connected to: %s\n", addr);

        setup_alert_service();
        update_led_state(BLE_STATE_CONNECTED);

        const struct bt_le_conn_param param = BT_LE_CONN_PARAM_INIT(24, 40, 0, 400);
        int rc = bt_conn_le_param_update(conn, &param);
        if (rc) {
            printf("conn param update failed (%d)\n", rc);
        }
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printf("Disconnected (reason %u)\n", reason);
    my_connection = NULL;
    update_led_state(BLE_STATE_ADVERTISING);
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = connected,
    .disconnected = disconnected,
};

int init_ble(void)
{
    int err;

    printf("Initializing BLE\n");
    printf("Calling bt_enable...\n");

    err = bt_enable(NULL);
    if (err) {
        printf("bt_enable failed (err %d)\n", err);
        return err;
    }
    printf("bt_enable done!\n");

    bt_conn_cb_register(&conn_callbacks);
    printf("conn callbacks registered\n");

    setup_alert_service();
    printf("alert service setup done\n");

    printf("BLE stack ready.\n");
    return 0;
}