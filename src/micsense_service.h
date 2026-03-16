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
#include <zephyr/sys/printk.h>

/* ──────────────────────────────────────────────────────────────
 * Compile‑time feature switches
 *
 *  - USE_STATUS_LED          : enable RGB status LED
 *  - USE_LOGGING             : enable runtime logging (printf-style)
 *  - USE_LOW_TX_POWER        : reduce BLE TX power (e.g. -8 dBm)
 *  - USE_LOW_RADIO_DUTY_CYCLE: longer conn interval/latency
 *  - USE_BLE                  : enable BLE stack (advertising, notifications)
 *
 * Define via compiler flags (e.g. -DUSE_LOGGING) or uncomment below.
 * LOG_PRINT(...) is the single macro for all logging: same format as
 * printf()/printk(); when USE_LOGGING is undefined it compiles to no-op.
 * ──────────────────────────────────────────────────────────── */

/* Uncomment to enable (or pass -DUSE_LOGGING etc. from build): */
// #define USE_LOGGING
// #define USE_STATUS_LED
// #define USE_LOW_TX_POWER
// #define USE_LOW_RADIO_DUTY_CYCLE
#define USE_BLE


#ifdef USE_LOGGING
#define LOG_PRINT(...) printk(__VA_ARGS__)
#else
#define LOG_PRINT(...) do { } while (0)
#endif

// GET_THRESHOLD_SERVICE=4fafc202-1fb5-459e-8fcc-c5c9c331914c
// GET_THRESHOLD_CHARACTERISTIC=beb5483f-36e1-4688-b7f5-ea07361b26a9
// 1. UUID VALUE DEFINITIONS (for BT_DATA_BYTES)
#define BT_UUID_GET_THRESHOLD_SERVICE_VAL        BT_UUID_128_ENCODE(0x4fafc202, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914c)
#define BT_UUID_GET_THRESHOLD_CHARACTERISTIC_VAL BT_UUID_128_ENCODE(0xbeb5483f, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26a9)

// SET_THRESHOLD_SERVICE=4fafc201-1fb5-459e-8fcc-c5c9c331914b
// SET_THRESHOLD_CHARACTERISTIC=beb5483e-36e1-4688-b7f5-ea07361b26a8
#define BT_UUID_SET_THRESHOLD_SERVICE_VAL        BT_UUID_128_ENCODE(0x4fafc201, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914b)
#define BT_UUID_SET_THRESHOLD_CHARACTERISTIC_VAL BT_UUID_128_ENCODE(0xbeb5483e, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26a8)

// GET_BATTERY_SERVICE=4fafc203-1fb5-459e-8fcc-c5c9c331914a
// GET_BATTERY_CHARACTERISTIC=beb54840-36e1-4688-b7f5-ea07361b26ab
#define BT_UUID_GET_BATTERY_SERVICE_VAL          BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914a)
#define BT_UUID_GET_BATTERY_CHARACTERISTIC_VAL   BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26ab)


// GET_SOUND_LEVEL_SERVICE=4fafc203-1fb5-459e-8fcc-c5c9c331914e
// GET_SOUND_LEVEL_CHARACTERISTIC=beb54840-36e1-4688-b7f5-ea07361b26ac
#define BT_UUID_GET_SOUND_LEVEL_SERVICE_VAL      BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914e)
#define BT_UUID_GET_SOUND_LEVEL_CHARACTERISTIC_VAL BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26ac)

// SET_SOUND_LEVEL_SERVICE=4fafc203-1fb5-459e-8fcc-c5c9c331914f
// SET_SOUND_LEVEL_CHARACTERISTIC=beb54840-36e1-4688-b7f5-ea07361b26ad
#define BT_UUID_SET_SOUND_LEVEL_SERVICE_VAL      BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914f)
#define BT_UUID_SET_SOUND_LEVEL_CHARACTERISTIC_VAL BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26ad)

// THRESHOLD_ALERT_SERVICE=4fafc203-1fb5-459e-8fcc-c5c9c331914d
// THRESHOLD_ALERT_CHARACTERISTIC=beb54840-36e1-4688-b7f5-ea07361b26aa
#define BT_UUID_THRESHOLD_ALERT_SERVICE_VAL      BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914d)
#define BT_UUID_THRESHOLD_ALERT_CHARACTERISTIC_VAL BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26aa)

// BABY_CRY_SERVICE=4fafc205-1fb5-459e-8fcc-c5c9c3319151
// BABY_CRY_CHARACTERISTIC=beb54842-36e1-4688-b7f5-ea07361b26af
#define BT_UUID_BABY_CRY_SERVICE_VAL             BT_UUID_128_ENCODE(0x4fafc205, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c3319151)
#define BT_UUID_BABY_CRY_CHARACTERISTIC_VAL      BT_UUID_128_ENCODE(0xbeb54842, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26af)

//
// int MICSENSE_service_init (void);
extern uint8_t r, g, b;
extern float db;
extern int16_t db_int;
extern uint8_t alertThreshold;
extern uint8_t threshold_value;
extern uint8_t sound_streaming_enabled;


extern struct bt_conn *my_connection;
extern bool notify_enabled;

extern const struct bt_gatt_attr *alert_threshold_attr;
extern const struct bt_gatt_attr *getStreamService_attr;
extern const struct bt_gatt_attr *baby_cry_attr;
extern uint8_t baby_cry_detected;
// extern volatile bool ble_ready = false;

enum ble_state {
    BLE_STATE_IDLE,
    BLE_STATE_ADVERTISING,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED
};

/* LED helpers (no-op when USE_STATUS_LED is not defined; implemented in main.c) */
void update_led_strip(uint8_t r, uint8_t g, uint8_t b);
void update_led_state(enum ble_state state);

/* Initialize the MICSENSE BLE service */
int MICSENSE_service_init(void);

/* Initialize BLE stack */
int init_ble(void);

/* BLE Connection Callbacks */
void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value);
