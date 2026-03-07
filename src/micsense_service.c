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

#define BT_UUID_AUDIO_SERVICE                    BT_UUID_DECLARE_128(BT_UUID_AUDIO_SERVICE_VAL)
#define BT_UUID_AUDIO_CONTROL_CHARACTERISTIC     BT_UUID_DECLARE_128(BT_UUID_AUDIO_CONTROL_CHAR_VAL)
#define BT_UUID_AUDIO_DATA_CHARACTERISTIC        BT_UUID_DECLARE_128(BT_UUID_AUDIO_DATA_CHAR_VAL)

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

volatile bool audio_recording = false;
volatile uint8_t audio_status = AUDIO_STATUS_IDLE;
uint16_t audio_actual_rate = AUDIO_SAMPLE_RATE;
static struct k_work_delayable audio_xfer_work;
static uint32_t audio_xfer_offset = 0;

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




// Audio recording service handlers
static ssize_t read_audio_status(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    void *buf,
    uint16_t len,
    uint16_t offset)
{
    // Return [status(1), rate_lo(1), rate_hi(1)] = 3 bytes
    uint8_t data[3];
    data[0] = audio_status;
    data[1] = (uint8_t)(audio_actual_rate & 0xFF);
    data[2] = (uint8_t)(audio_actual_rate >> 8);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

static ssize_t on_audio_control(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t offset,
    uint8_t flags)
{
    uint8_t cmd;
    memcpy(&cmd, buf, sizeof(uint8_t));

    switch (cmd) {
    case AUDIO_CMD_START_REC:
        printk("Audio: Start recording\n");
        audio_write_index = 0;
        audio_recording = true;
        audio_status = AUDIO_STATUS_RECORDING;
        break;
    case AUDIO_CMD_STOP_REC:
        printk("Audio: Stop recording (%u samples)\n", audio_write_index);
        audio_recording = false;
        audio_status = AUDIO_STATUS_IDLE;
        break;
    case AUDIO_CMD_TRANSFER:
        printk("Audio: Transfer requested (%u samples)\n", audio_write_index);
        audio_recording = false;
        audio_transfer_start();
        break;
    default:
        printk("Audio: Unknown command 0x%02x\n", cmd);
        break;
    }
    return len;
}

static void audio_xfer_handler(struct k_work *work)
{
    if (!my_connection) {
        audio_status = AUDIO_STATUS_IDLE;
        return;
    }

    uint32_t total_bytes = audio_write_index * sizeof(int16_t);
    if (audio_xfer_offset >= total_bytes) {
        audio_status = AUDIO_STATUS_IDLE;
        printk("Audio: Transfer complete\n");
        return;
    }

    // Determine max chunk data size from current ATT MTU
    uint16_t mtu = bt_gatt_get_mtu(my_connection);
    if (mtu < 23) {
        mtu = 23;
    }
    uint16_t max_notify_payload = mtu - 3;   // ATT notification header
    uint16_t chunk_data_max = max_notify_payload - 4;  // our 4-byte header

    uint8_t buf[244];  // max possible (MTU 247 - 3)
    uint16_t total_chunks = (total_bytes + chunk_data_max - 1) / chunk_data_max;
    uint16_t chunk_index = audio_xfer_offset / chunk_data_max;
    uint32_t remaining = total_bytes - audio_xfer_offset;
    uint16_t chunk_len = (remaining < chunk_data_max) ? (uint16_t)remaining : chunk_data_max;

    memcpy(buf, &chunk_index, 2);
    memcpy(buf + 2, &total_chunks, 2);
    memcpy(buf + 4, ((uint8_t *)audio_buffer) + audio_xfer_offset, chunk_len);

    int err = bt_gatt_notify(my_connection, audio_data_attr, buf, chunk_len + 4);
    if (err) {
        printk("Audio: chunk %u notify failed (%d), MTU=%u, retrying\n",
               chunk_index, err, mtu);
        k_work_reschedule(&audio_xfer_work, K_MSEC(100));
        return;
    }

    audio_xfer_offset += chunk_len;

    if (audio_xfer_offset >= total_bytes) {
        audio_status = AUDIO_STATUS_IDLE;
        printk("Audio: Transfer complete (%u chunks, MTU=%u)\n", total_chunks, mtu);
    } else {
        k_work_reschedule(&audio_xfer_work, K_MSEC(15));
    }
}

void audio_transfer_start(void)
{
    if (audio_write_index == 0) {
        printk("Audio: No data to transfer\n");
        return;
    }
    audio_status = AUDIO_STATUS_TRANSFERRING;
    audio_xfer_offset = 0;
    k_work_reschedule(&audio_xfer_work, K_MSEC(10));
}

void audio_init(void)
{
    k_work_init_delayable(&audio_xfer_work, audio_xfer_handler);
}

const struct bt_gatt_attr *audio_data_attr;

BT_GATT_SERVICE_DEFINE(audioSrvc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_AUDIO_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_CONTROL_CHARACTERISTIC,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                           BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                           read_audio_status, on_audio_control, &audio_status),
    BT_GATT_CHARACTERISTIC(BT_UUID_AUDIO_DATA_CHARACTERISTIC,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

void setup_alert_service(void)
{
    alert_threshold_attr = &alertThresholdSrvc.attrs[1];
    getStreamService_attr = &getStreamService.attrs[1];
    baby_cry_attr = &babyCrySrvc.attrs[1];
    audio_data_attr = &audioSrvc.attrs[3];
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
