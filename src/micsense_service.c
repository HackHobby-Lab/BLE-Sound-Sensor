#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include "micsense_service.h"
/* UUIDs Definitions */
// / 2. UUID DECLARATIONS (for GATT)
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

#define BT_UUID_BABY_CRY_SERVICE                 BT_UUID_DECLARE_128(BT_UUID_BABY_CRY_SERVICE_VAL)
#define BT_UUID_BABY_CRY_CHARACTERISTIC          BT_UUID_DECLARE_128(BT_UUID_BABY_CRY_CHARACTERISTIC_VAL)

#define BT_UUID_MICSENESE BT_UUID_DECLARE_128(BT_UUID_MICSENESE_VAL)
#define MAX_TRANSMIT_SIZE 1024
volatile bool ble_ready = false;

struct bt_conn *my_connection = NULL;
bool notify_enabled = false;

uint8_t threshold_value  = 50;    // Example threshold
static uint8_t battery_level = 40;       // Simulated battery %
static uint8_t sound_level = 0;          // Simulated sound
uint8_t sound_streaming_enabled = 0;
static bool threshold_alert_enabled = false;
uint8_t alertThreshold;
uint8_t baby_cry_detected = 0;           // Baby cry detection status (0 or 1)


int MICSENSE_service_init(void)
{
    
}// Notify the client after a successful write operation


static ssize_t on_receive(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr,
                          const void *buf,
                          uint16_t len,
                          uint16_t offset,
                          uint8_t flags)
{
    uint8_t received_value;

    // Safely copy the received buffer into the integer
    memcpy(&received_value, buf, sizeof(uint8_t));
    threshold_value = received_value;
    printk("Received integer: %d\n", received_value);
    printk("------Value stored in threshold variable: %d\n", threshold_value);
    return len;  // Return the length of data written
}

static ssize_t on_SetStream(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags)
{
uint8_t received_value;

// Safely copy the received buffer into the integer
memcpy(&received_value, buf, sizeof(uint8_t));
sound_streaming_enabled = received_value;
printk("Received Set Stream: %d\n", received_value);
return len;  // Return the length of data written
}


ssize_t on_getThreshold(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset)
{
printk("In On get Threshold function\n");
return bt_gatt_attr_read(conn, attr, buf,len, offset, &threshold_value, sizeof(threshold_value)); // Handle reading logic here if needed
};

void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch (value)
    {
    case BT_GATT_CCC_NOTIFY:
        printk("Notifications enabled\n");
        notify_enabled = true;
        printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
        break;
    case BT_GATT_CCC_INDICATE:
        // Handle indications if necessary
        break;
    case 0:
        printk("Notifications disabled\n");
        notify_enabled = false;
        printk("Notifications %s\n", notify_enabled ? "enabled" : "disabled");
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

static ssize_t read_baby_cry(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    void *buf,
    uint16_t len,
    uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &baby_cry_detected, sizeof(uint8_t));
}

// GATT service definition with read/write characteristics
BT_GATT_SERVICE_DEFINE(setThreshold,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_SET_THRESHOLD_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_SET_THRESHOLD_CHARACTERISTIC,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, on_receive, NULL),  // Write characteristic
                       );
// Write sound level status to device (1 to start getting sound level, 0 to stop getting sound level)
// SET_SOUND_LEVEL_SERVICE
// SET_SOUND_LEVEL_CHARACTERISTIC
// const struct bt_gatt_attr *getStreamService_attr;
BT_GATT_SERVICE_DEFINE(setStreamService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SET_SOUND_LEVEL_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_SET_SOUND_LEVEL_CHARACTERISTIC,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL,on_SetStream,  NULL),  // Read/Notify characteristic
    
    // BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);


BT_GATT_SERVICE_DEFINE(getThreshold,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_GET_THRESHOLD_SERVICE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_GET_THRESHOLD_CHARACTERISTIC,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_hello, NULL, &threshold_value),  // Read/Notify characteristic
                       
                    //    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

BT_GATT_SERVICE_DEFINE(batteryService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GET_BATTERY_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_GET_BATTERY_CHARACTERISTIC,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &battery_level),  // Read/Notify characteristic
    
    // BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

const struct bt_gatt_attr *alert_threshold_attr;

BT_GATT_SERVICE_DEFINE(alertThresholdSrvc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_THRESHOLD_ALERT_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_THRESHOLD_ALERT_CHARACTERISTIC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &alertThreshold),  // Read/Notify characteristic
    
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);




// Read current sound level from the device(Stream/Notification)
// GET_SOUND_LEVEL_SERVICE
// GET_SOUND_LEVEL_CHARACTERISTIC

const struct bt_gatt_attr *getStreamService_attr;
BT_GATT_SERVICE_DEFINE(getStreamService,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_GET_SOUND_LEVEL_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_GET_SOUND_LEVEL_CHARACTERISTIC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_hello, NULL, &db_int),  // Read/Notify characteristic
    
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

// Baby cry detection alert service
const struct bt_gatt_attr *baby_cry_attr;
BT_GATT_SERVICE_DEFINE(babyCrySrvc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_BABY_CRY_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_BABY_CRY_CHARACTERISTIC,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_baby_cry, NULL, &baby_cry_detected),  // Read/Notify characteristic
    
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);




void setup_alert_service(void)
{
    alert_threshold_attr = &alertThresholdSrvc.attrs[1];
    getStreamService_attr = &getStreamService.attrs[1];
    baby_cry_attr = &babyCrySrvc.attrs[1];
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
        printf("Connection established! Connected to: %s Role: %u Connection interval: %u Slave latency: %u Connection supervisory timeout: %u\n",
               addr, info.role, info.le.interval, info.le.latency, info.le.timeout);

        setup_alert_service();
        update_led_state(BLE_STATE_CONNECTED);

        /* 🔑 Request safe connection parameters */
        const struct bt_le_conn_param *param = BT_LE_CONN_PARAM(24, 40, 0, 400);

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
    .connected = connected,
    .disconnected = disconnected};

void bt_ready(int err)
{
    if (err)
    {
        printf("BT_ENABLE RETURN %d\n", err);
    }
    printf("BT ENABLE\n");
    ble_ready = true;
    bt_conn_cb_register(&conn_callbacks);
}

int init_ble(void)
{
    printf("Initializing BLE\n");
    int err;
    err = bt_enable(bt_ready);
    if (err)
    {
        printf("Bt Enable failed with error code (code %d)\n", err);
        return err;
    }
    return 0;
}
