#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SAMPLE_FREQ        48000
#define SAMPLE_BIT_WIDTH   24
#define BYTES_PER_SAMPLE   4         /* nRF I2S stores each 24-bit sample in a 32-bit memory word */
#define NUM_CHANNELS       2
#define FRAMES_PER_BLOCK   512
#define BLOCK_SIZE         (FRAMES_PER_BLOCK * NUM_CHANNELS * BYTES_PER_SAMPLE)
#define BLOCK_COUNT        4
#define I2S_TIMEOUT_MS     2000

#define BLOCKS_PER_PRINT   3
#define BAR_WIDTH          40
#define DBFS_FLOOR        -80.0
#define ICS43434_OFFSET    120.0

/* Phase-1 data collection toggle. 1 = emit CSV log over UART;
 * 0 = production mode (silent UART, runs cry detection + alerts).
 */
#define DATA_COLLECT_MODE  0
#define BLOCKS_PER_LOG     10           /* ~107 ms per CSV row in data mode */

/* First-order IIR band-split coefficients (fs = 48 kHz).
 *   low band: LPF @ 200 Hz   (HVAC, fan hum)
 *   mid band: HPF @ 200 Hz  ->  LPF @ 1500 Hz  (baby-cry energy)
 */
#define ALPHA_LP_200       0.025513f
#define ALPHA_LP_1500      0.164138f
#define ALPHA_HP_200       0.974487f

/* Cry-detection defaults (overridden by NVS-persisted values if present).
 * Detection rule:
 *   loud      = (db_spl       >= threshold_value)
 *   voice     = (dbfs_mid - dbfs_low) >= voice_gap_min
 *   sustained = loud && voice held continuously for >= debounce_seconds
 * Once triggered, a notification fires every alert_period_s for window_seconds.
 * If still loud+voice when the window ends, the window restarts.
 */
#define THRESHOLD_DEFAULT            70u   /* dB SPL */
#define CAL_OFFSET_DEFAULT            0    /* signed dB trim on ICS43434_OFFSET */
#define VOICE_GAP_MIN_DEFAULT         5u   /* dB, mid - low */
#define DEBOUNCE_SECONDS_DEFAULT      3u
#define WINDOW_SECONDS_DEFAULT       30u
#define ALERT_PERIOD_SECONDS_DEFAULT  3u

/* Sound-level calibration (mirrors version 3.0 "JackJack" behaviour).
 * The app writes 1 to the Calibrate characteristic; firmware averages
 * ~2 s of ambient SPL and trims cal_offset_db so the quiet room reads
 * CAL_TARGET_DB. This keeps the app's threshold slider on the same
 * scale it used with the production firmware.
 */
#define CAL_TARGET_DB                35    /* calibrated quiet room reads this */
#define CAL_SAMPLES                  19    /* ~2 s at ~107 ms / iteration */

/* Mic-Sense service / characteristic UUIDs (must match the mobile app). */
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

/* Calibrate + Get Noise Floor — UUIDs must match version 3.0 firmware
 * so the mobile app's service discovery sees an identical GATT table.
 */
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

K_MEM_SLAB_DEFINE_STATIC(mic_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* IIR filter state. Continuous across blocks — never reset mid-stream. */
static float lp_low_y;          /* LPF200 output (low band)              */
static float hp_mid_y_prev;     /* HPF200 previous output                */
static float hp_mid_x_prev;     /* HPF200 previous input                 */
static float lp_mid_y;          /* LPF1500 of HPF output (mid band)      */

/* GATT state — types match the original Mic-Sense firmware. */
static uint8_t  threshold_value          = THRESHOLD_DEFAULT;
static uint8_t  sound_streaming_enabled  = 1;    /* 1 = stream notifications on */
static uint8_t  alertThreshold           = 0;    /* 1 = currently in a cry event */
static uint8_t  battery_level            = 100;  /* mock — DK is USB-powered */
static int16_t  db_int                   = 0;    /* current SPL value (signed int) */
static int8_t   cal_offset_db            = CAL_OFFSET_DEFAULT;  /* per-unit SPL trim */
static uint8_t  noise_floor_db           = 0;    /* calibrated quiet-room dB (0 = uncalibrated) */
static volatile uint8_t calibration_requested = 0;  /* set by Calibrate characteristic write */

/* Tunables — could be moved to NVS later. */
static uint8_t  voice_gap_min     = VOICE_GAP_MIN_DEFAULT;
static uint8_t  debounce_seconds  = DEBOUNCE_SECONDS_DEFAULT;
static uint8_t  window_seconds    = WINDOW_SECONDS_DEFAULT;
static uint8_t  alert_period_s    = ALERT_PERIOD_SECONDS_DEFAULT;

static bool stream_notify_enabled;
static bool alert_notify_enabled;

/* ---------- Cry-detection state machine ---------- */
typedef enum {
	DET_IDLE,       /* not above threshold or not voice-like  */
	DET_RISING,     /* above threshold, counting toward 2s    */
	DET_ALERTING,   /* in active alert window, sending notify */
} det_state_t;

static det_state_t det_state;
static int64_t     det_rise_start_ms;
static int64_t     det_last_loud_ms;     /* most recent loud_voice=1 cycle */
static int64_t     det_window_start_ms;
static int64_t     det_last_alert_ms;

/* While in RISING, allow this much continuous silence (loud_voice=0) before
 * resetting the timer. Real cries / yells fluctuate at the cycle level and
 * shouldn't bounce back to IDLE on every micro-pause.
 */
#define RISING_GRACE_MS  700

/* ---------- Persistent settings (NVS via Zephyr settings subsystem) ---------- */

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

/* ---------- GATT read / write callbacks ---------- */

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

/* Writes that should survive a power cycle queue a save to NVS. */
static ssize_t write_u8_persisted(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ssize_t r = write_u8(conn, attr, buf, len, offset, flags);
	if (r >= 0) {
		k_work_submit(&settings_save_work);
	}
	return r;
}

/* App writes 1 here to request a sound-level calibration run. The actual
 * 2 s ambient measurement happens in the main loop (see calibration block).
 */
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

/* ---------- GATT services ---------- */

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

/* Calibrate service — app writes 1 to trigger a 2 s ambient measurement
 * that re-trims the dB scale (quiet room -> CAL_TARGET_DB). Matches
 * version 3.0 firmware so the mobile app's "calibrate" button works.
 */
BT_GATT_SERVICE_DEFINE(calibrate_svc,
	BT_GATT_PRIMARY_SERVICE(&calibrate_svc_uuid),
	BT_GATT_CHARACTERISTIC(&calibrate_chr_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, write_calibrate, NULL),
);

/* Get Noise Floor service — app reads the calibrated quiet-room dB level
 * (0 until a calibration has been run). Matches version 3.0 firmware.
 */
BT_GATT_SERVICE_DEFINE(noise_floor_svc,
	BT_GATT_PRIMARY_SERVICE(&noise_floor_svc_uuid),
	BT_GATT_CHARACTERISTIC(&noise_floor_chr_uuid.uuid,
		BT_GATT_CHRC_READ,
		BT_GATT_PERM_READ,
		read_u8, NULL, &noise_floor_db),
);

/* ---------- Advertising ---------- */

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
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected    = on_connected,
	.disconnected = on_disconnected,
};

static int ble_start(void)
{
	int err = bt_enable(NULL);
	if (err) {
		printk("bt_enable failed: %d\n", err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
			      adv_data, ARRAY_SIZE(adv_data),
			      scan_rsp, ARRAY_SIZE(scan_rsp));
	if (err) {
		printk("bt_le_adv_start failed: %d\n", err);
		return err;
	}

	printk("BLE advertising as \"Mic-Sense\"\n");
	return 0;
}

/* ---------- I2S audio path ---------- */

static int configure_i2s(const struct device *dev)
{
	struct i2s_config cfg = {
		.word_size      = SAMPLE_BIT_WIDTH,
		.channels       = NUM_CHANNELS,
		.format         = I2S_FMT_DATA_FORMAT_I2S,
		.options        = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER,
		.frame_clk_freq = SAMPLE_FREQ,
		.mem_slab       = &mic_slab,
		.block_size     = BLOCK_SIZE,
		.timeout        = I2S_TIMEOUT_MS,
	};
	return i2s_configure(dev, I2S_DIR_RX, &cfg);
}

struct block_sums {
	double full_sq;
	double low_sq;
	double mid_sq;
	size_t n;
};

static void compute_block_sums(const int32_t *samples, size_t total_samples,
			       struct block_sums *out)
{
	double full = 0.0, low = 0.0, mid = 0.0;
	size_t n = 0;
	const float inv_scale = 1.0f / (float)(1 << 23);

	/* ICS43434 L/R pin = GND → data on left channel only.
	 * Zephyr's nRF I2S driver delivers the 24-bit sample sign-extended
	 * in the low 24 bits of the 32-bit word.
	 */
	for (size_t i = 0; i < total_samples; i += NUM_CHANNELS) {
		float x = (float)samples[i] * inv_scale;

		lp_low_y = ALPHA_LP_200 * x + (1.0f - ALPHA_LP_200) * lp_low_y;

		float hp = ALPHA_HP_200 * (hp_mid_y_prev + x - hp_mid_x_prev);
		hp_mid_y_prev = hp;
		hp_mid_x_prev = x;
		lp_mid_y = ALPHA_LP_1500 * hp + (1.0f - ALPHA_LP_1500) * lp_mid_y;

		full += (double)(x        * x);
		low  += (double)(lp_low_y * lp_low_y);
		mid  += (double)(lp_mid_y * lp_mid_y);
		n++;
	}

	out->full_sq = full;
	out->low_sq  = low;
	out->mid_sq  = mid;
	out->n       = n;
}

static void print_signed_decimal(int x10)
{
	int a = abs(x10);
	printk("%s%d.%d", (x10 < 0) ? "-" : "", a / 10, a % 10);
}

static void render_bar(double dbfs, char *out)
{
	double frac = (dbfs - DBFS_FLOOR) / (0.0 - DBFS_FLOOR);
	if (frac < 0.0) frac = 0.0;
	if (frac > 1.0) frac = 1.0;
	int filled = (int)(frac * BAR_WIDTH + 0.5);

	for (int i = 0; i < BAR_WIDTH; i++) {
		out[i] = (i < filled) ? '#' : '-';
	}
	out[BAR_WIDTH] = '\0';
}

int main(void)
{
	const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));

	if (!device_is_ready(i2s_dev)) {
		printk("ERROR: I2S device not ready\n");
		return -1;
	}

	int err = configure_i2s(i2s_dev);
	if (err) {
		printk("ERROR: i2s_configure: %d\n", err);
		return err;
	}

	err = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (err) {
		printk("ERROR: i2s_trigger START: %d\n", err);
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

#if DATA_COLLECT_MODE
	printk("\n# Mic-Sense data collection  |  %d Hz, %d-bit, left channel\n",
	       SAMPLE_FREQ, SAMPLE_BIT_WIDTH);
	printk("# bands: low=LPF<200Hz, mid=BPF 200-1500Hz, full=raw\n");
	printk("# t_ms,dbfs,db_spl,dbfs_low,dbfs_mid\n");
#else
	printk("\nMic-Sense ready  |  %d Hz, %d-bit, left channel\n",
	       SAMPLE_FREQ, SAMPLE_BIT_WIDTH);
	printk("Cry detect: SPL>=%u dB AND mid-low gap>=%u dB sustained %us\n",
	       threshold_value, voice_gap_min, debounce_seconds);
	printk("Alerts: every %us for %us window (auto-restart while loud)\n",
	       alert_period_s, window_seconds);
	printk("Calibration offset: %d dB  (noise floor: %d dB)\n\n",
	       (int)cal_offset_db, (int)noise_floor_db);
#endif

	struct block_sums acc = { 0 };
	int blocks = 0;
	const int blocks_per_print = BLOCKS_PER_LOG;

	while (1) {
		void *buf;
		size_t size;

		err = i2s_read(i2s_dev, &buf, &size);
		if (err) {
			printk("\ni2s_read err %d — restarting\n", err);
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_PREPARE);
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
			continue;
		}

		struct block_sums b;
		compute_block_sums((const int32_t *)buf,
				   size / BYTES_PER_SAMPLE, &b);
		acc.full_sq += b.full_sq;
		acc.low_sq  += b.low_sq;
		acc.mid_sq  += b.mid_sq;
		acc.n       += b.n;
		k_mem_slab_free(&mic_slab, buf);

		if (++blocks < blocks_per_print) {
			continue;
		}

		double inv_n   = (acc.n > 0) ? 1.0 / (double)acc.n : 0.0;
		double rms     = sqrt(acc.full_sq * inv_n);
		double rms_low = sqrt(acc.low_sq  * inv_n);
		double rms_mid = sqrt(acc.mid_sq  * inv_n);

		double dbfs     = (rms     > 1e-9) ? 20.0 * log10(rms)     : -120.0;
		double dbfs_low = (rms_low > 1e-9) ? 20.0 * log10(rms_low) : -120.0;
		double dbfs_mid = (rms_mid > 1e-9) ? 20.0 * log10(rms_mid) : -120.0;
		double spl      = dbfs + ICS43434_OFFSET + (double)cal_offset_db;

		int dbfs_x10     = (int)(dbfs     * 10.0);
		int spl_x10      = (int)(spl      * 10.0);
		int dbfs_low_x10 = (int)(dbfs_low * 10.0);
		int dbfs_mid_x10 = (int)(dbfs_mid * 10.0);

#if DATA_COLLECT_MODE
		printk("%u,", k_uptime_get_32());
		print_signed_decimal(dbfs_x10);     printk(",");
		print_signed_decimal(spl_x10);      printk(",");
		print_signed_decimal(dbfs_low_x10); printk(",");
		print_signed_decimal(dbfs_mid_x10); printk("\n");
#else
		(void)dbfs_x10;
		(void)spl_x10;
#endif

		int spl_rounded = (int)(spl + 0.5);
		if (spl_rounded < INT16_MIN) spl_rounded = INT16_MIN;
		if (spl_rounded > INT16_MAX) spl_rounded = INT16_MAX;
		db_int = (int16_t)spl_rounded;

		if (stream_notify_enabled && sound_streaming_enabled) {
			(void)bt_gatt_notify(NULL, &get_sound_level_svc.attrs[1],
					     &db_int, sizeof(db_int));
		}

		/* Sound-level calibration. When the app writes 1 to the
		 * Calibrate characteristic, average ~2 s of ambient SPL and
		 * set cal_offset_db so the quiet room reads CAL_TARGET_DB.
		 * Result is persisted to NVS.
		 */
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

		/* Cry-detection state machine.
		 * Trigger when SPL >= threshold AND mid-low gap >= voice_gap_min,
		 * sustained for debounce_seconds. Then notify every alert_period_s
		 * for window_seconds; restart window if still loud at the end.
		 */
		int gap_x10    = dbfs_mid_x10 - dbfs_low_x10;
		bool loud      = (db_int >= (int16_t)threshold_value);
		bool voice     = (gap_x10 >= (int)voice_gap_min * 10);
		bool loud_voice = loud && voice;
		int64_t now_ms = k_uptime_get();

		/* Once-per-second status so we can see live SPL/gap/state
		 * even when no cry has triggered. Remove after tuning is done.
		 */
#if !DATA_COLLECT_MODE
		{
			static int stat_div = 0;
			if (++stat_div >= 10) {
				stat_div = 0;
				const char *state_str =
					(det_state == DET_IDLE)   ? "IDLE"   :
					(det_state == DET_RISING) ? "RISING" :
								    "ALERT";
				printk("[STAT] SPL=%d gap=%d (loud=%d voice=%d) state=%s\n",
				       (int)db_int, gap_x10 / 10,
				       (int)loud, (int)voice, state_str);
			}
		}
#endif

		switch (det_state) {
		case DET_IDLE:
			if (loud_voice) {
				det_rise_start_ms = now_ms;
				det_last_loud_ms  = now_ms;
				det_state = DET_RISING;
			}
			break;

		case DET_RISING:
			if (loud_voice) {
				det_last_loud_ms = now_ms;
			} else if (now_ms - det_last_loud_ms > RISING_GRACE_MS) {
				/* Quiet too long — abandon rise */
				det_state = DET_IDLE;
				break;
			}
			if (now_ms - det_rise_start_ms >=
			    (int64_t)debounce_seconds * 1000) {
				det_window_start_ms = now_ms;
				det_last_alert_ms   = now_ms;
				alertThreshold = 1;
				if (alert_notify_enabled) {
					(void)bt_gatt_notify(NULL,
						&threshold_alert_svc.attrs[1],
						&alertThreshold, sizeof(alertThreshold));
				}
				printk("[CRY] event triggered (SPL=%d, gap=%d dB)\n",
				       (int)db_int, gap_x10 / 10);
				det_state = DET_ALERTING;
			}
			break;

		case DET_ALERTING:
			if (now_ms - det_last_alert_ms >=
			    (int64_t)alert_period_s * 1000) {
				det_last_alert_ms = now_ms;
				if (alert_notify_enabled) {
					(void)bt_gatt_notify(NULL,
						&threshold_alert_svc.attrs[1],
						&alertThreshold, sizeof(alertThreshold));
				}
			}
			if (now_ms - det_window_start_ms >=
			    (int64_t)window_seconds * 1000) {
				if (loud_voice) {
					det_window_start_ms = now_ms;
					printk("[CRY] window restart (still loud)\n");
				} else {
					alertThreshold = 0;
					if (alert_notify_enabled) {
						(void)bt_gatt_notify(NULL,
							&threshold_alert_svc.attrs[1],
							&alertThreshold, sizeof(alertThreshold));
					}
					printk("[CRY] event ended\n");
					det_state = DET_IDLE;
				}
			}
			break;
		}

		acc.full_sq = 0.0;
		acc.low_sq  = 0.0;
		acc.mid_sq  = 0.0;
		acc.n       = 0;
		blocks      = 0;
	}
	return 0;
}
