/* ============================================================================
 *  Mic-Sense — MAX9814 (analog) build
 *
 *  This is the SECOND microphone path. It is identical to the ICS-43434 build
 *  in every respect (BLE GATT contract, button/power latch, cry-detection
 *  state machine, calibration, RTT console) EXCEPT the audio front-end:
 *
 *     ICS-43434 build : digital I2S mic, 24-bit samples
 *     MAX9814  build  : analog mic read through the nRF52833 SAADC (ADC)
 *
 *  Hardware (Sound Sensor V0.3, NINA-B406 / nRF52833):
 *     MOUTPUT (MAX9814 MICOUT) = NINA GPIO_18 = nRF P0.02 = ADC AIN0
 *     (confirmed in NINA-B40 datasheet Table 6: pin 18 -> P0.02, analog capable)
 *
 *  Sliders on the board configure the MAX9814 itself (no firmware involvement):
 *     AGC  -> OFF   (fixed gain, so loudness is preserved for the detector)
 *     GAIN -> 40 dB (lowest, so a loud cry doesn't clip)
 *
 *  IMPORTANT — calibration: the absolute dB scale of an analog mic depends on
 *  its gain/sensitivity, so MAX9814_OFFSET below is only a starting estimate.
 *  Run the app's Calibrate button once in a quiet room (anchors the quiet
 *  level to 35 dB) and re-check the threshold — exactly the same one-time
 *  step you did for the ICS-43434 build.
 * ========================================================================== */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <hal/nrf_saadc.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ─── Audio capture: MAX9814 analog mic via SAADC on AIN0 (P0.02) ─── */
#define ADC_NODE              DT_NODELABEL(adc)
#define ADC_RESOLUTION        12
#define ADC_CHANNEL_ID        0
#define ADC_INPUT_POS         NRF_SAADC_INPUT_AIN0   /* AIN0 = P0.02 = MOUTPUT */

#define SAMPLE_RATE_HZ        4000                   /* uniform, timer-paced */
#define SAMPLE_INTERVAL_US    (1000000 / SAMPLE_RATE_HZ)   /* 250 us */
#define WINDOW_SAMPLES        400                    /* ~100 ms per window */

/* dBFS reference: a signal centred in the 12-bit single-ended range can swing
 * +/- 2048 counts, so 2048 counts is "full scale" for the AC component. */
#define ADC_FULLSCALE_COUNTS  2048.0

/* Starting dB offset (analog mic). Calibration re-anchors the quiet level;
 * this just puts the raw scale in a sensible ballpark before that. Tune after
 * measuring real levels, same as the ICS build's ICS43434_OFFSET. */
#define MAX9814_OFFSET        95.0

#define DBFS_FLOOR           -80.0

/* ─────────────────────────────────────────────────────────────────────
 *  Cry detection — escalation rules (identical to the ICS-43434 build)
 * ───────────────────────────────────────────────────────────────────── */
#define THRESHOLD_DEFAULT            75u   /* dB SPL */
#define CAL_OFFSET_DEFAULT            0    /* signed dB trim on MAX9814_OFFSET */
#define VOICE_GAP_MIN_DEFAULT         0u
#define DEBOUNCE_SECONDS_DEFAULT      2u   /* continuous cry needed before alerting */
#define WINDOW_SECONDS_DEFAULT       30u
#define ALERT_PERIOD_SECONDS_DEFAULT  3u   /* notify cadence once alerting */

/* Once a cry is confirmed, keep sending an alert every ALERT_PERIOD_SECONDS
 * for this long — even if the baby goes quiet (the "2 minutes" rule). */
#define ALERT_LATCH_SECONDS         120u

#define NAG_AFTER_TRIGGERS            3u
#define HYSTERESIS_DB                 5
#define IDLE_RESET_SECONDS           30u
#define RISING_GRACE_MS             700    /* tolerated micro-pauses while confirming */

/* Calibration: app writes 1 to the Calibrate characteristic; firmware averages
 * ~2 s of ambient SPL and trims cal_offset_db so a quiet room reads
 * CAL_TARGET_DB. Keeps the app threshold slider on a consistent scale. */
#define CAL_TARGET_DB                35
#define CAL_SAMPLES                  19    /* ~1.9 s at ~100 ms / window */

/* ─── Battery fuel gauge: BQ27426, I2C @ 0x55 (kept, currently disabled) ─── */
#define BQ27426_I2C_ADDR       0x55
#define BQ27426_CMD_VOLTAGE    0x04
#define BQ27426_CMD_SOC        0x1C
#define BATTERY_POLL_CYCLES    90

/* ─── Power latch: EN_CONTROLL = NINA GPIO_5 = nRF P0.17 ─── */
#define EN_CONTROLL_PORT       DT_NODELABEL(gpio0)
#define EN_CONTROLL_PIN        17

/* ─── Main button: PAIR_BUTTON = NINA GPIO_42 = nRF P0.26 ───
 * Active-HIGH with external pull-down (R146). Short press = toggle
 * advertising; long press (>= BUTTON_LONG_PRESS_MS) = power off. */
#define PAIR_BUTTON_PORT       DT_NODELABEL(gpio0)
#define PAIR_BUTTON_PIN        26
#define BUTTON_LONG_PRESS_MS   3000
#define BUTTON_DEBOUNCE_MS     40

/* ─── Mic-Sense service / characteristic UUIDs (must match the mobile app) ─── */
#define BT_UUID_SET_THRESHOLD_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc201, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914bULL)
#define BT_UUID_SET_THRESHOLD_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb5483e, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26a8ULL)

#define BT_UUID_GET_THRESHOLD_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc202, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914cULL)
#define BT_UUID_GET_THRESHOLD_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb5483f, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26a9ULL)

#define BT_UUID_GET_BATTERY_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914aULL)
#define BT_UUID_GET_BATTERY_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26abULL)

#define BT_UUID_GET_SOUND_LEVEL_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914eULL)
#define BT_UUID_GET_SOUND_LEVEL_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26acULL)

#define BT_UUID_SET_SOUND_LEVEL_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914fULL)
#define BT_UUID_SET_SOUND_LEVEL_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26adULL)

#define BT_UUID_THRESHOLD_ALERT_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914dULL)
#define BT_UUID_THRESHOLD_ALERT_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb54840, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26aaULL)

#define BT_UUID_CALIBRATE_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc204, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c3319150ULL)
#define BT_UUID_CALIBRATE_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb54841, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26aeULL)

#define BT_UUID_NOISE_FLOOR_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc204, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c3319151ULL)
#define BT_UUID_NOISE_FLOOR_CHR_VAL \
	BT_UUID_128_ENCODE(0xbeb54841, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26afULL)

static struct bt_uuid_128 set_threshold_svc_uuid    = BT_UUID_INIT_128(BT_UUID_SET_THRESHOLD_SVC_VAL);
static struct bt_uuid_128 set_threshold_chr_uuid    = BT_UUID_INIT_128(BT_UUID_SET_THRESHOLD_CHR_VAL);
static struct bt_uuid_128 get_threshold_svc_uuid    = BT_UUID_INIT_128(BT_UUID_GET_THRESHOLD_SVC_VAL);
static struct bt_uuid_128 get_threshold_chr_uuid    = BT_UUID_INIT_128(BT_UUID_GET_THRESHOLD_CHR_VAL);
static struct bt_uuid_128 get_battery_svc_uuid      = BT_UUID_INIT_128(BT_UUID_GET_BATTERY_SVC_VAL);
static struct bt_uuid_128 get_battery_chr_uuid      = BT_UUID_INIT_128(BT_UUID_GET_BATTERY_CHR_VAL);
static struct bt_uuid_128 get_sound_level_svc_uuid  = BT_UUID_INIT_128(BT_UUID_GET_SOUND_LEVEL_SVC_VAL);
static struct bt_uuid_128 get_sound_level_chr_uuid  = BT_UUID_INIT_128(BT_UUID_GET_SOUND_LEVEL_CHR_VAL);
static struct bt_uuid_128 set_sound_level_svc_uuid  = BT_UUID_INIT_128(BT_UUID_SET_SOUND_LEVEL_SVC_VAL);
static struct bt_uuid_128 set_sound_level_chr_uuid  = BT_UUID_INIT_128(BT_UUID_SET_SOUND_LEVEL_CHR_VAL);
static struct bt_uuid_128 threshold_alert_svc_uuid  = BT_UUID_INIT_128(BT_UUID_THRESHOLD_ALERT_SVC_VAL);
static struct bt_uuid_128 threshold_alert_chr_uuid  = BT_UUID_INIT_128(BT_UUID_THRESHOLD_ALERT_CHR_VAL);
static struct bt_uuid_128 calibrate_svc_uuid        = BT_UUID_INIT_128(BT_UUID_CALIBRATE_SVC_VAL);
static struct bt_uuid_128 calibrate_chr_uuid        = BT_UUID_INIT_128(BT_UUID_CALIBRATE_CHR_VAL);
static struct bt_uuid_128 noise_floor_svc_uuid      = BT_UUID_INIT_128(BT_UUID_NOISE_FLOOR_SVC_VAL);
static struct bt_uuid_128 noise_floor_chr_uuid      = BT_UUID_INIT_128(BT_UUID_NOISE_FLOOR_CHR_VAL);

/* ─── ADC objects ─── */
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static int16_t adc_buf[WINDOW_SAMPLES];

static const struct adc_channel_cfg adc_ch_cfg = {
	.gain             = ADC_GAIN_1_6,        /* input range 0..3.6 V with 0.6 V ref */
	.reference        = ADC_REF_INTERNAL,    /* 0.6 V internal reference */
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id       = ADC_CHANNEL_ID,
	.input_positive   = ADC_INPUT_POS,
};

static const struct adc_sequence_options adc_opts = {
	.interval_us     = SAMPLE_INTERVAL_US,   /* kernel-timer paced, yields to BLE */
	.extra_samplings = WINDOW_SAMPLES - 1,   /* total = 1 + extra = WINDOW_SAMPLES */
	.callback        = NULL,
};

static struct adc_sequence adc_seq = {
	.options     = &adc_opts,
	.channels    = BIT(ADC_CHANNEL_ID),
	.buffer      = adc_buf,
	.buffer_size = sizeof(adc_buf),
	.resolution  = ADC_RESOLUTION,
};

/* ─── GATT state ─── */
static uint8_t  threshold_value          = THRESHOLD_DEFAULT;
static uint8_t  sound_streaming_enabled  = 1;
static uint8_t  alertThreshold           = 0;
static uint8_t  battery_level            = 100;
static int16_t  db_int                   = 0;
static int8_t   cal_offset_db            = CAL_OFFSET_DEFAULT;
static uint8_t  noise_floor_db           = 0;
static volatile uint8_t calibration_requested = 0;

static uint8_t  voice_gap_min     = VOICE_GAP_MIN_DEFAULT;
static uint8_t  debounce_seconds  = DEBOUNCE_SECONDS_DEFAULT;
static uint8_t  window_seconds    = WINDOW_SECONDS_DEFAULT;
static uint8_t  alert_period_s    = ALERT_PERIOD_SECONDS_DEFAULT;

static bool stream_notify_enabled;
static bool alert_notify_enabled;

/* ─── Cry-detection state machine ─── */
typedef enum {
	DET_IDLE,
	DET_RISING,
	DET_ACTIVE,
} det_state_t;

static det_state_t det_state;
static int64_t     det_rise_start_ms;
static int64_t     det_last_loud_ms;
static int64_t     det_last_alert_ms;
static int64_t     det_alert_start_ms;
static int64_t     det_quiet_since_ms;
static uint8_t     det_trigger_count;
static bool        det_dipped_since_alert;

/* ─── Persistent settings (NVS via Zephyr settings subsystem) ─── */
static int micsense_settings_set(const char *name, size_t len,
				 settings_read_cb read_cb, void *cb_arg)
{
	ssize_t r;

	if (!strcmp(name, "threshold")) {
		r = read_cb(cb_arg, &threshold_value, sizeof(threshold_value));
		return (r < 0) ? (int)r : 0;
	}
	if (!strcmp(name, "cal_offset")) {
		r = read_cb(cb_arg, &cal_offset_db, sizeof(cal_offset_db));
		return (r < 0) ? (int)r : 0;
	}
	if (!strcmp(name, "noise_floor")) {
		r = read_cb(cb_arg, &noise_floor_db, sizeof(noise_floor_db));
		return (r < 0) ? (int)r : 0;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(micsense, "micsense", NULL,
			       micsense_settings_set, NULL, NULL);

static void settings_save_handler(struct k_work *w)
{
	(void)settings_save_one("micsense/threshold",
				&threshold_value, sizeof(threshold_value));
	(void)settings_save_one("micsense/cal_offset",
				&cal_offset_db, sizeof(cal_offset_db));
	(void)settings_save_one("micsense/noise_floor",
				&noise_floor_db, sizeof(noise_floor_db));
}

static K_WORK_DEFINE(settings_save_work, settings_save_handler);

/* ─── GATT read / write callbacks ─── */
static ssize_t read_u8(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 attr->user_data, sizeof(uint8_t));
}

static ssize_t read_i16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 attr->user_data, sizeof(int16_t));
}

static ssize_t write_u8(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *target = attr->user_data;

	if (offset + len > sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(target + offset, buf, len);
	return len;
}

static ssize_t write_u8_persisted(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ssize_t r = write_u8(conn, attr, buf, len, offset, flags);
	if (r >= 0) {
		printk("[APP] threshold updated to %u dB\n", (unsigned)threshold_value);
		k_work_submit(&settings_save_work);
	}
	return r;
}

static ssize_t write_calibrate(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (((const uint8_t *)buf)[0] == 1) {
		calibration_requested = 1;
		printk("[CAL] calibration requested via BLE\n");
	}
	return len;
}

static void sound_level_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("[CCC] sound-level notify %s\n",
	       stream_notify_enabled ? "ENABLED" : "disabled");
}

static void alert_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	alert_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("[CCC] alert notify %s\n",
	       alert_notify_enabled ? "ENABLED" : "disabled");
}

/* ─── GATT services (identical table to the ICS-43434 build) ─── */
BT_GATT_SERVICE_DEFINE(set_threshold_svc,
	BT_GATT_PRIMARY_SERVICE(&set_threshold_svc_uuid),
	BT_GATT_CHARACTERISTIC(&set_threshold_chr_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, write_u8_persisted, &threshold_value),
);

BT_GATT_SERVICE_DEFINE(get_threshold_svc,
	BT_GATT_PRIMARY_SERVICE(&get_threshold_svc_uuid),
	BT_GATT_CHARACTERISTIC(&get_threshold_chr_uuid.uuid,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_u8, NULL, &threshold_value),
);

BT_GATT_SERVICE_DEFINE(battery_svc,
	BT_GATT_PRIMARY_SERVICE(&get_battery_svc_uuid),
	BT_GATT_CHARACTERISTIC(&get_battery_chr_uuid.uuid,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_u8, NULL, &battery_level),
);

BT_GATT_SERVICE_DEFINE(get_sound_level_svc,
	BT_GATT_PRIMARY_SERVICE(&get_sound_level_svc_uuid),
	BT_GATT_CHARACTERISTIC(&get_sound_level_chr_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_i16, NULL, &db_int),
	BT_GATT_CCC(sound_level_ccc_changed,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

BT_GATT_SERVICE_DEFINE(set_sound_level_svc,
	BT_GATT_PRIMARY_SERVICE(&set_sound_level_svc_uuid),
	BT_GATT_CHARACTERISTIC(&set_sound_level_chr_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, write_u8, &sound_streaming_enabled),
);

BT_GATT_SERVICE_DEFINE(threshold_alert_svc,
	BT_GATT_PRIMARY_SERVICE(&threshold_alert_svc_uuid),
	BT_GATT_CHARACTERISTIC(&threshold_alert_chr_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_u8, NULL, &alertThreshold),
	BT_GATT_CCC(alert_ccc_changed,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

BT_GATT_SERVICE_DEFINE(calibrate_svc,
	BT_GATT_PRIMARY_SERVICE(&calibrate_svc_uuid),
	BT_GATT_CHARACTERISTIC(&calibrate_chr_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, write_calibrate, NULL),
);

BT_GATT_SERVICE_DEFINE(noise_floor_svc,
	BT_GATT_PRIMARY_SERVICE(&noise_floor_svc_uuid),
	BT_GATT_CHARACTERISTIC(&noise_floor_chr_uuid.uuid,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_u8, NULL, &noise_floor_db),
);

/* ─── Advertising ─── */
static const struct bt_data adv_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SET_THRESHOLD_SVC_VAL),
};

static const struct bt_data scan_rsp[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, "Mic-Sense", 9),
};

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("\nBLE connect failed (0x%02x)\n", err);
		return;
	}
	printk("\nBLE connected\n");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("\nBLE disconnected (0x%02x)\n", reason);
	stream_notify_enabled = false;
	alert_notify_enabled  = false;
	det_state = DET_IDLE;
	det_trigger_count = 0;
	det_dipped_since_alert = false;
	det_quiet_since_ms = 0;
	alertThreshold = 0;
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected    = on_connected,
	.disconnected = on_disconnected,
};

static bool advertising_on;

static int adv_start(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
				  adv_data, ARRAY_SIZE(adv_data),
				  scan_rsp, ARRAY_SIZE(scan_rsp));
	if (err) {
		printk("bt_le_adv_start failed: %d\n", err);
		return err;
	}
	advertising_on = true;
	printk("BLE advertising as \"Mic-Sense\"\n");
	return 0;
}

static void adv_stop(void)
{
	(void)bt_le_adv_stop();
	advertising_on = false;
	printk("BLE advertising stopped\n");
}

static void button_short_press(void)
{
	if (advertising_on) {
		adv_stop();
	} else {
		(void)adv_start();
	}
}

static void power_off(void)
{
	const struct device *en_port = DEVICE_DT_GET(EN_CONTROLL_PORT);

	printk("\n[PWR] long press — powering off\n");

	if (advertising_on) {
		(void)bt_le_adv_stop();
	}
	if (device_is_ready(en_port)) {
		gpio_pin_set(en_port, EN_CONTROLL_PIN, 0);
	}
	while (1) {
		k_sleep(K_SECONDS(1));
	}
}

static int ble_start(void)
{
	int err = bt_enable(NULL);
	if (err) {
		printk("bt_enable failed: %d\n", err);
		return err;
	}
	return adv_start();
}

/* ─── Audio: one SAADC window -> SPL in dB ───
 * Removes the mic's DC bias by computing the variance (AC power) of the
 * window, so we don't need to know the exact bias point. */
static double window_spl(void)
{
	int64_t sum = 0, sumsq = 0;

	for (int i = 0; i < WINDOW_SAMPLES; i++) {
		int32_t s = adc_buf[i];
		sum   += s;
		sumsq += (int64_t)s * s;
	}

	double n      = (double)WINDOW_SAMPLES;
	double mean   = (double)sum / n;
	double meansq = (double)sumsq / n;
	double var    = meansq - mean * mean;   /* AC power, DC removed */
	if (var < 0.0) {
		var = 0.0;
	}
	double rms  = sqrt(var);
	double dbfs = (rms > 1e-6) ? 20.0 * log10(rms / ADC_FULLSCALE_COUNTS)
				   : DBFS_FLOOR;
	return dbfs + MAX9814_OFFSET + (double)cal_offset_db;
}

/* ─── Alert helper ─── */
static void fire_alert(void)
{
	alertThreshold = 1;
	if (alert_notify_enabled) {
		int rc = bt_gatt_notify(NULL, &threshold_alert_svc.attrs[1],
					&alertThreshold, sizeof(alertThreshold));
		printk("[CRY] ALERT #%u FIRED (SPL=%d dB) notify_rc=%d\n",
		       (unsigned)det_trigger_count, (int)db_int, rc);
	} else {
		printk("[CRY] ALERT #%u suppressed — app hasn't enabled CCC for the alert characteristic\n",
		       (unsigned)det_trigger_count);
	}
}

/* ─── Battery fuel gauge (BQ27426 over I2C) — kept, currently disabled ─── */
static const struct device *batt_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static int bq27426_read16(uint8_t cmd, uint16_t *out)
{
	uint8_t rx[2];
	int err = i2c_write_read(batt_i2c, BQ27426_I2C_ADDR,
				 &cmd, 1, rx, sizeof(rx));
	if (err) {
		return err;
	}
	*out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
	return 0;
}

static void battery_poll(void)
{
	uint16_t soc, mv;

	if (!device_is_ready(batt_i2c)) {
		return;
	}
	if (bq27426_read16(BQ27426_CMD_SOC, &soc) == 0) {
		if (soc > 100) {
			soc = 100;
		}
		battery_level = (uint8_t)soc;
		if (bq27426_read16(BQ27426_CMD_VOLTAGE, &mv) == 0) {
			printk("[BATT] %u%%  (%u mV)\n",
			       (unsigned)battery_level, (unsigned)mv);
		} else {
			printk("[BATT] %u%%\n", (unsigned)battery_level);
		}
	} else {
		printk("[BATT] gauge not responding (no BQ27426 on I2C?) — holding %u%%\n",
		       (unsigned)battery_level);
	}
}

/* ─── Main button poll (non-blocking) ─── */
static void button_poll(void)
{
	static const struct device *btn_port;
	static bool prev_pressed;
	static int64_t press_start_ms;
	static bool long_fired;

	if (btn_port == NULL) {
		btn_port = DEVICE_DT_GET(PAIR_BUTTON_PORT);
		if (!device_is_ready(btn_port)) {
			btn_port = NULL;
			return;
		}
	}

	bool pressed = gpio_pin_get(btn_port, PAIR_BUTTON_PIN) == 1;
	int64_t now = k_uptime_get();

	if (pressed && !prev_pressed) {
		press_start_ms = now;
		long_fired = false;
	} else if (pressed && prev_pressed) {
		if (!long_fired &&
		    (now - press_start_ms) >= BUTTON_LONG_PRESS_MS) {
			long_fired = true;
			power_off();          /* does not return */
		}
	} else if (!pressed && prev_pressed) {
		int64_t held = now - press_start_ms;
		if (!long_fired && held >= BUTTON_DEBOUNCE_MS) {
			button_short_press();
		}
	}

	prev_pressed = pressed;
}

int main(void)
{
	/* Latch board power first: drive EN_CONTROLL HIGH before anything else. */
	{
		const struct device *en_port = DEVICE_DT_GET(EN_CONTROLL_PORT);
		if (device_is_ready(en_port)) {
			gpio_pin_configure(en_port, EN_CONTROLL_PIN, GPIO_OUTPUT_HIGH);
			printk("Power latch: EN_CONTROLL (P0.%02d) held HIGH\n",
			       EN_CONTROLL_PIN);
		} else {
			printk("WARN: EN_CONTROLL GPIO port not ready\n");
		}
	}

	/* Main button input (external pull-down on the board; press = HIGH). */
	{
		const struct device *btn_port = DEVICE_DT_GET(PAIR_BUTTON_PORT);
		if (device_is_ready(btn_port)) {
			gpio_pin_configure(btn_port, PAIR_BUTTON_PIN, GPIO_INPUT);
			printk("Main button on P0.%02d (short=pairing, long %us=off)\n",
			       PAIR_BUTTON_PIN, BUTTON_LONG_PRESS_MS / 1000);
		}
	}

	/* ADC (MAX9814 analog mic on AIN0 / P0.02). */
	if (!device_is_ready(adc_dev)) {
		printk("ERROR: ADC device not ready\n");
		return -1;
	}
	int err = adc_channel_setup(adc_dev, &adc_ch_cfg);
	if (err) {
		printk("ERROR: adc_channel_setup: %d\n", err);
		return err;
	}

	err = settings_subsys_init();
	if (err) {
		printk("settings_subsys_init failed: %d\n", err);
	} else {
		err = settings_load();
		if (err) {
			printk("settings_load failed: %d\n", err);
		}
	}

	(void)ble_start();

	if (!device_is_ready(batt_i2c)) {
		printk("WARN: I2C bus for fuel gauge not ready — battery stays at default\n");
	}

	printk("\nMic-Sense (MAX9814 / SAADC) ready  |  %d Hz, AIN0 = P0.02\n",
	       SAMPLE_RATE_HZ);
	printk("Sliders: set AGC = OFF, GAIN = 40 dB on the board.\n");
	printk("Alert rule:\n");
	printk("  trigger -> SPL >= %u dB for %us continuous\n",
	       threshold_value, debounce_seconds);
	printk("  then    -> notify every %us for %us (even if it goes quiet)\n",
	       alert_period_s, ALERT_LATCH_SECONDS);
	printk("  reset   -> window ends; re-arms if still crying\n");
	printk("Calibration offset: %d dB  (noise floor: %d dB)\n",
	       (int)cal_offset_db, (int)noise_floor_db);
	printk("NOTE: run the app's Calibrate once in a quiet room, then check threshold.\n\n");

	bool first_read = true;

	while (1) {
		/* Calibrate the SAADC offset on the very first read (temperature
		 * compensation); cheap and improves accuracy. */
		adc_seq.calibrate = first_read;
		err = adc_read(adc_dev, &adc_seq);
		first_read = false;
		if (err) {
			printk("\nadc_read err %d — retrying\n", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		double spl = window_spl();

		int spl_rounded = (int)(spl + 0.5);
		if (spl_rounded < INT16_MIN) spl_rounded = INT16_MIN;
		if (spl_rounded > INT16_MAX) spl_rounded = INT16_MAX;
		db_int = (int16_t)spl_rounded;

		if (stream_notify_enabled && sound_streaming_enabled) {
			(void)bt_gatt_notify(NULL, &get_sound_level_svc.attrs[1],
					     &db_int, sizeof(db_int));
		}

		/* Sound-level calibration: average ~2 s of ambient and set
		 * cal_offset_db so a quiet room reads CAL_TARGET_DB. */
		if (calibration_requested) {
			static int    cal_n;
			static double cal_sum;
			double spl_raw = spl - (double)cal_offset_db;

			cal_sum += spl_raw;
			cal_n++;
			if (cal_n >= CAL_SAMPLES) {
				double ambient = cal_sum / (double)cal_n;
				int off = (int)((double)CAL_TARGET_DB - ambient + 0.5);
				if (off < -128) off = -128;
				if (off >  127) off =  127;
				cal_offset_db  = (int8_t)off;
				noise_floor_db = CAL_TARGET_DB;
				k_work_submit(&settings_save_work);
				printk("[CAL] done: ambient=%d dB raw -> offset=%d dB\n",
				       (int)ambient, (int)cal_offset_db);
				cal_n   = 0;
				cal_sum = 0.0;
				calibration_requested = 0;
			}
		}

		bool loud      = (db_int >= (int16_t)threshold_value);
		int64_t now_ms = k_uptime_get();

		/* Once-per-second status line (~10 windows at ~100 ms each). */
		{
			static int stat_div = 0;
			if (++stat_div >= 10) {
				stat_div = 0;
				const char *state_str =
					(det_state == DET_IDLE)   ? "IDLE"   :
					(det_state == DET_RISING) ? "RISING" :
								    "ACTIVE";
				printk("[STAT] SPL=%d thr=%u state=%s "
				       "alerts=%u stream_sub=%d alert_sub=%d\n",
				       (int)db_int, threshold_value, state_str,
				       (unsigned)det_trigger_count,
				       (int)stream_notify_enabled,
				       (int)alert_notify_enabled);
			}
		}

		switch (det_state) {
		case DET_IDLE:
			if (loud) {
				det_rise_start_ms = now_ms;
				det_last_loud_ms  = now_ms;
				det_state = DET_RISING;
				printk("[CRY] rising — SPL=%d, need %us continuous\n",
				       (int)db_int, debounce_seconds);
			}
			break;

		case DET_RISING:
			if (loud) {
				det_last_loud_ms = now_ms;
			} else if (now_ms - det_last_loud_ms > RISING_GRACE_MS) {
				det_state = DET_IDLE;
				printk("[CRY] rising aborted (quiet during the %us debounce)\n",
				       debounce_seconds);
				break;
			}
			if (now_ms - det_rise_start_ms >=
			    (int64_t)debounce_seconds * 1000) {
				det_alert_start_ms = now_ms;
				det_trigger_count  = 1;
				fire_alert();
				det_last_alert_ms  = now_ms;
				det_state          = DET_ACTIVE;
				printk("[CRY] CONFIRMED — alerting every %us for the next %us\n",
				       alert_period_s, ALERT_LATCH_SECONDS);
			}
			break;

		case DET_ACTIVE:
			if (now_ms - det_alert_start_ms >=
			    (int64_t)ALERT_LATCH_SECONDS * 1000) {
				alertThreshold = 0;
				if (alert_notify_enabled) {
					(void)bt_gatt_notify(NULL,
						&threshold_alert_svc.attrs[1],
						&alertThreshold,
						sizeof(alertThreshold));
				}
				printk("[CRY] %us alert window elapsed — stopping\n",
				       ALERT_LATCH_SECONDS);
				det_state         = DET_IDLE;
				det_trigger_count = 0;
				break;
			}
			if (now_ms - det_last_alert_ms >=
			    (int64_t)alert_period_s * 1000) {
				det_trigger_count++;
				fire_alert();
				det_last_alert_ms = now_ms;
			}
			break;
		}

		/* Main button: short press = pairing toggle, long press = off. */
		button_poll();

		/* Battery fuel-gauge poll — DISABLED (blocking I2C drops BLE).
		 * Re-enable later as a non-blocking work-queue task. */
#if 0
		{
			static int batt_div = BATTERY_POLL_CYCLES;
			if (++batt_div >= BATTERY_POLL_CYCLES) {
				batt_div = 0;
				battery_poll();
			}
		}
#endif
	}
	return 0;
}
