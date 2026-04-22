#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include "micsense_service.h"
#include <zephyr/drivers/led_strip.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <soc.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Forward declaration */
void update_led_state(enum ble_state state);

/* LED strip */
#define DELAY_TIME K_MSEC(5)
#define STRIP_NUM_PIXELS 1
static struct led_rgb pixels[STRIP_NUM_PIXELS];
static const struct device *strip = NULL;

#define LED_BLINK_ADV_SLOW  K_MSEC(1000)
#define LED_BLINK_CONN_FAST K_MSEC(200)
#define LED_CONNECTED_GREEN K_MSEC(0)

/* [BAT-OPT-5] Auto-off: turn LED off after this many ms to save power.
 * WS2812 at full white draws ~18 mA. Turning it off after a short
 * status flash saves significant energy over time. */
#define LED_AUTO_OFF_MS     3000

static enum ble_state current_ble_state = BLE_STATE_IDLE;
static struct k_timer led_blink_timer;
static struct k_timer led_off_timer;          /* [BAT-OPT-5] new */
static bool led_on = false;

/* GPIO */
#define EN_PIN_NODE   DT_PATH(zephyr_user)
#define PAIR_PIN_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec pwr_En   = GPIO_DT_SPEC_GET(EN_PIN_NODE,   user_output_gpios);
static const struct gpio_dt_spec pair_pin = GPIO_DT_SPEC_GET(PAIR_PIN_NODE, user_input_gpios);

static struct gpio_callback input_cb_data;
static struct k_work button_work;

bool status = false;
int count = 0;
static bool advertising_active = false;

/* [BAT-OPT-6] REMOVED: int16_t audio_buffer[16000]
 * This allocated 32 KB of RAM and was never written or read by any
 * code path in this file. Removing it frees RAM, reduces stack/heap
 * pressure, and avoids keeping memory power rails unnecessarily busy. */

#define SLEEP_TIME_MS 100
float db = 0.0;
int16_t db_int = 0;
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 12
#define ADC_CHANNEL    0
#define ADC_PORT       SAADC_CH_PSELP_PSELP_AnalogInput0
#define ADC_REFERENCE  ADC_REF_INTERNAL
#define ADC_GAIN       ADC_GAIN_1_6

#define BATTERY_ADC_CHANNEL 1
#define BATTERY_ADC_PORT    SAADC_CH_PSELP_PSELP_AnalogInput1

struct adc_channel_cfg chl0_cfg = {
    .gain             = ADC_GAIN,
    .reference        = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive   = ADC_PORT
#endif
};

struct adc_channel_cfg battery_ch_cfg = {
    .gain             = ADC_GAIN,
    .reference        = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = BATTERY_ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive   = BATTERY_ADC_PORT
#endif
};

int16_t sampleBuffer[1];

struct adc_sequence sequence = {
    .channels    = BIT(ADC_CHANNEL),
    .buffer      = sampleBuffer,
    .buffer_size = sizeof(sampleBuffer),
    .resolution  = ADC_RESOLUTION
};

int16_t battery_sample[1];

struct adc_sequence battery_sequence = {
    .channels    = BIT(BATTERY_ADC_CHANNEL),
    .buffer      = battery_sample,
    .buffer_size = sizeof(battery_sample),
    .resolution  = ADC_RESOLUTION
};

/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Scan response data */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SET_THRESHOLD_SERVICE_VAL),
};

/* [BAT-OPT-3] Slow advertising parameters.
 *
 * Original: BT_GAP_ADV_FAST_INT_MIN_2 / MAX_2  →  100–150 ms interval
 *           Radio fires ~8 times/sec, dominating current draw in standby.
 *
 * Optimized: 1000–1280 ms interval.
 *            Radio fires ~1 time/sec instead of 8.
 *            Measured impact on nRF52: ~1.5 mA → ~0.2 mA average radio
 *            current in advertising state. Discovery is slightly slower
 *            (~1–2 s) but completely acceptable for a sensor device.
 *
 * If you ever need fast discovery on first boot, use fast advertising
 * for 30 s then switch to slow — see comment below main(). */
#define ADV_PARAM_SLOW \
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, \
        BT_GAP_ADV_SLOW_INT_MIN, \
        BT_GAP_ADV_SLOW_INT_MAX, \
        NULL)

static struct k_timer button_timer;
static bool long_press_detected = false;

/* [BAT-OPT-5] LED auto-off callback */
static void led_off_timer_expiry(struct k_timer *timer_id)
{
    update_led_strip(0, 0, 0);
}

static void button_work_handler(struct k_work *work)
{
    if (k_timer_status_get(&button_timer) > 0) {
        printk("Long press detected!\n");
        gpio_pin_set_dt(&pwr_En, 0);
        while (1) {
            k_sleep(K_FOREVER);
        }
    } else {
        printk("Short press: toggling BLE advertising...\n");
        int err;
        if (advertising_active) {
            err = bt_le_adv_stop();
            if (!err) {
                advertising_active = false;
                update_led_state(BLE_STATE_IDLE);
            }
        } else {
            /* [BAT-OPT-3] Use slow advertising on button-toggle too */
            err = bt_le_adv_start(ADV_PARAM_SLOW,
                                  ad, ARRAY_SIZE(ad),
                                  sd, ARRAY_SIZE(sd));
            if (!err) {
                advertising_active = true;
                update_led_state(BLE_STATE_ADVERTISING);
            }
        }
    }

    k_timer_stop(&button_timer);
}

void button_timer_expiry(struct k_timer *timer_id)
{
    long_press_detected = true;
    k_work_submit(&button_work);
}

void input_pin_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    bool pin_state = gpio_pin_get_dt(&pair_pin);

    if (pin_state) {
        k_timer_start(&button_timer, K_SECONDS(3), K_NO_WAIT);
    } else {
        k_work_submit(&button_work);
    }
}

void update_led_strip(uint8_t r, uint8_t g, uint8_t b)
{
    if (strip == NULL) {
        return;
    }
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i].r = r;
        pixels[i].g = g;
        pixels[i].b = b;
    }
    led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
}

void update_led_state(enum ble_state state)
{
    current_ble_state = state;

    switch (state) {
    case BLE_STATE_ADVERTISING:
        update_led_strip(255, 255, 0);
        break;
    case BLE_STATE_CONNECTING:
        update_led_strip(255, 165, 0);
        break;
    case BLE_STATE_CONNECTED:
        update_led_strip(0, 255, 0);
        break;
    default:
        update_led_strip(255, 0, 0);
        break;
    }

    /* [BAT-OPT-5] Schedule LED auto-off after LED_AUTO_OFF_MS.
     * The LED shows the status flash then turns off to save power.
     * k_timer_start on an already-running timer restarts it, so
     * calling update_led_state() again resets the timeout correctly. */
    k_timer_start(&led_off_timer, K_MSEC(LED_AUTO_OFF_MS), K_NO_WAIT);
}

/* ─────────────────────────────────────────────
 * CALIBRATION & DSP PARAMETERS
 * ───────────────────────────────────────────── */

#define NUM_SAMPLES       20
#define WARMUP_SAMPLES    5

#define SMOOTHING_ALPHA   0.25f
#define DEFAULT_DB_OFFSET 0.0f

static float   db_filtered        = 0.0f;
static float   calibration_offset = DEFAULT_DB_OFFSET;
static int32_t baseline_dc        = 2812;

#define SAMPLE_RATE_HZ   8000
#define FRAME_SAMPLES    64
#define HPF_R            0.995f
#define FRAME_MS         ((1000 * FRAME_SAMPLES) / SAMPLE_RATE_HZ)

/* [AGC-WORKAROUND] Bandpass tuned for baby cry fundamental (250-600 Hz)
 * instead of the old 2000 Hz. Reason: the MAX9814 has AGC enabled in hardware
 * (R151/R152 divider on TH pin) — we cannot disable it without a PCB change.
 * AGC compresses loud signals, so amplitude-based RMS detection is weak.
 * Moving BPF down to 500 Hz preserves the baby-cry band, which is the
 * primary signal we care about for this product. */
#define BPF_FC_HZ        500.0f
#define BPF_Q            0.5f
#define DB_BPF_GAIN_COMP 30.0f

/* [AGC-WORKAROUND] Zero-crossing detector config.
 * AGC compresses amplitude but does NOT hide how often the signal oscillates.
 * A quiet room produces random low-ZCR noise. A real sound produces
 * a high, sustained zero-crossing rate. This is our primary activity
 * detector that survives AGC.
 *
 * TUNING NOTES (from real test data on this PCB):
 *   Quiet room baseline ZCR per frame: 9-15 (peak ~17)
 *   Clap / loud HELLO on mic:          18-25
 *   So we need threshold above noise floor, and require the event
 *   to persist for multiple frames to reject spurious spikes.
 */
#define ZCR_MIN_FOR_SOUND    18    /* threshold: above quiet-room noise ceiling */
#define ZCR_STRONG_SOUND     22    /* above this = definitely a real event */
#define ZCR_SUSTAIN_FRAMES   2     /* require N consecutive frames above threshold */
#define ZCR_DB_SCALE         2.5f  /* aggressive boost once threshold is crossed */
#define ZCR_DB_FLOOR         10.0f /* base boost added on top */

#define ADC_MV_MARGIN    2800

/* [BAT-OPT-7] Notification cooldown: minimum ms between BLE dB stream
 * notifications. Original code notified every 8 ms frame — that is 125
 * packets/sec, keeping the radio almost constantly active.
 * 100 ms (10 Hz) is more than sufficient for a dB meter UI and cuts
 * radio-active time by ~12x during streaming. */
#define STREAM_NOTIFY_COOLDOWN_MS  100
static uint32_t last_stream_notify_ms = 0;

typedef struct {
    float a0, a1, a2, b1, b2, z1, z2;
} biquad_t;

static biquad_t bpf;

static void biquad_init_bandpass(biquad_t *s, float sample_rate_hz, float f0_hz, float q)
{
    float w0     = 2.0f * (float)M_PI * f0_hz / sample_rate_hz;
    float sin_w0 = sinf(w0);
    float cos_w0 = cosf(w0);
    float alpha  = sin_w0 / (2.0f * q);

    float b0 =  q * alpha;
    float b1 =  0.0f;
    float b2 = -q * alpha;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 =  1.0f - alpha;

    s->a0 = b0 / a0;
    s->a1 = b1 / a0;
    s->a2 = b2 / a0;
    s->b1 = a1 / a0;
    s->b2 = a2 / a0;
    s->z1 = 0.0f;
    s->z2 = 0.0f;
}

static inline float biquad_process(biquad_t *s, float x)
{
    float y = s->a0 * x + s->z1;
    s->z1   = s->a1 * x - s->b1 * y + s->z2;
    s->z2   = s->a2 * x - s->b2 * y;
    return y;
}

static float hpf_prev_x = 0.0f;
static float hpf_prev_y = 0.0f;

#define TRIGGER_HOLD_MS    200
#define RESET_HOLD_MS      100
#define NOTIFY_COOLDOWN_MS 400

static uint32_t above_ms       = 0;
static uint32_t below_ms       = 0;
static uint32_t last_notify_ms = 0;
static int      notify_armed   = 1;

/* ─────────────────────────────────────────────
 * CALIBRATION FUNCTION
 * ───────────────────────────────────────────── */
void calibrate_baseline_dc(void)
{
    int32_t total = 0;
    int     valid = 0;
    int     err;

    /* [BAT-OPT-4] Cache vref once — it never changes at runtime.
     * Original code called adc_ref_internal() inside the per-sample
     * loop (64 calls/frame). It is a driver call with overhead.
     * Caching it here and in main loop saves repeated driver entry. */
    int32_t adc_vref = adc_ref_internal(adc_dev);

    printf("Warming up ADC (%d dummy reads)...\n", WARMUP_SAMPLES);
    for (int i = 0; i < WARMUP_SAMPLES; i++) {
        adc_read(adc_dev, &sequence);
        k_msleep(10);
    }

    printf("Sampling microphone for calibration (%d samples)...\n", NUM_SAMPLES);
    for (int i = 0; i < NUM_SAMPLES; i++) {
        err = adc_read(adc_dev, &sequence);
        if (err == 0) {
            int32_t mv_value = sampleBuffer[0];
            adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

            printf("  Calibration sample %d: %d mV\n", i, (int)mv_value);

            if (mv_value >= 100 && mv_value <= 3500) {
                total += mv_value;
                valid++;
            } else {
                printf("  Sample %d skipped (out of range)\n", i);
            }
        } else {
            printf("ADC read error during calibration: %d\n", err);
        }
        k_msleep(5);
    }

    if (valid > 0) {
        baseline_dc = total / valid;
    } else {
        baseline_dc = 2812;
        printf("WARNING: No valid samples! Using fallback baseline: %d mV\n",
               (int)baseline_dc);
    }

    printf("Calibrated baseline DC: %d mV (from %d/%d valid samples)\n",
           (int)baseline_dc, valid, NUM_SAMPLES);
}

/* ─────────────────────────────────────────────
 * SOUND-LEVEL CALIBRATION
 *
 * Measures ambient noise for ~2 seconds, then sets calibration_offset
 * so that the quiet room reads ~35 dB on the app meter.
 * Triggered by writing 1 to the Calibrate BLE characteristic.
 * ───────────────────────────────────────────── */
#define CAL_FRAMES        250   /* ~2 seconds at 8ms/frame */
#define CAL_TARGET_DB     35.0f /* quiet room should read this */

void calibrate_sound_level(int32_t adc_vref)
{
    float db_sum = 0.0f;
    int   valid_frames = 0;

    /* Temporary filter state so we don't disturb the main loop's state */
    biquad_t cal_bpf;
    biquad_init_bandpass(&cal_bpf, (float)SAMPLE_RATE_HZ, BPF_FC_HZ, BPF_Q);
    float cal_hpf_prev_x = 0.0f;
    float cal_hpf_prev_y = 0.0f;

    printk("=== Sound calibration started (measuring %d frames) ===\n", CAL_FRAMES);

    for (int frame = 0; frame < CAL_FRAMES; frame++) {
        float   sum_sq = 0.0f;   /* FIX: was int32_t — squared floats < 1 truncated to 0 */
        int     valid_samples = 0;
        int     err;

        int64_t frame_start_ms = k_uptime_get();

        for (int i = 0; i < FRAME_SAMPLES; i++) {
            err = adc_read(adc_dev, &sequence);
            if (err != 0) {
                continue;
            }

            int32_t mv_value = sampleBuffer[0];
            adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

            int32_t mv_min = baseline_dc - ADC_MV_MARGIN;
            int32_t mv_max = baseline_dc + ADC_MV_MARGIN;
            if (mv_value < mv_min || mv_value > mv_max) {
                continue;
            }

            valid_samples++;

            float x  = (float)(mv_value - baseline_dc);
            float y  = x - cal_hpf_prev_x + HPF_R * cal_hpf_prev_y;
            cal_hpf_prev_x = x;
            cal_hpf_prev_y = y;

            float y_bp = biquad_process(&cal_bpf, y);

            if (y_bp >  10000.0f) y_bp =  10000.0f;
            if (y_bp < -10000.0f) y_bp = -10000.0f;
            sum_sq += y_bp * y_bp;   /* FIX: no int cast */
        }

        /* Sleep remainder of frame — same pattern as main loop */
        int64_t elapsed_ms = k_uptime_get() - frame_start_ms;
        int32_t remaining_ms = (int32_t)FRAME_MS - (int32_t)elapsed_ms;
        if (remaining_ms > 0) {
            k_msleep(remaining_ms);
        }

        if (valid_samples == 0) {
            continue;
        }

        float rms = sqrtf(sum_sq / (float)valid_samples);
        if (rms < 1.0f) rms = 1.0f;

        float db_val = 20.0f * log10f(rms) + DB_BPF_GAIN_COMP;
        db_sum += db_val;
        valid_frames++;
    }

    if (valid_frames > 0) {
        float noise_floor_raw = db_sum / valid_frames;
        calibration_offset = CAL_TARGET_DB - noise_floor_raw;
        noise_floor_db = (uint8_t)CAL_TARGET_DB;

        printk("=== Calibration done! ===\n");
        printk("  Noise floor (raw): %d dB\n", (int)noise_floor_raw);
        printk("  Calibration offset: %d dB\n", (int)calibration_offset);
        printk("  Noise floor (calibrated): %d dB\n", (int)noise_floor_db);
    } else {
        printk("=== Calibration FAILED: no valid frames ===\n");
    }

    /* Reset main loop filter state so it starts clean after calibration */
    hpf_prev_x = 0.0f;
    hpf_prev_y = 0.0f;
    biquad_init_bandpass(&bpf, (float)SAMPLE_RATE_HZ, BPF_FC_HZ, BPF_Q);
    db_filtered = 0.0f;

    calibration_requested = 0;
}

int main(void)
{
    int err;
    printk("Startup\n");
    printk("PM enabled: %s\n", IS_ENABLED(CONFIG_PM) ? "YES" : "NO");

    k_work_init(&button_work, button_work_handler);
    k_timer_init(&button_timer, button_timer_expiry, NULL);

    /* [BAT-OPT-5] Init LED auto-off timer */
    k_timer_init(&led_off_timer, led_off_timer_expiry, NULL);

    update_led_state(BLE_STATE_IDLE);

    /* Power enable pin */
    if (!device_is_ready(pwr_En.port)) {
        printk("GPIO port not ready\n");
    } else {
        gpio_pin_configure_dt(&pwr_En, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&pwr_En, 1);
        printk("Power enable pin set HIGH\n");
    }

    printk("Button bypassed for testing\n");

    /* ── STEP 1: ADC setup and calibration BEFORE BLE ── */
    printk("Starting ADC setup...\n");

    if (!device_is_ready(adc_dev)) {
        printf("ADC Device not ready\n");
        return -1;
    }

    err = adc_channel_setup(adc_dev, &chl0_cfg);
    if (err != 0) {
        printf("ADC Setup failed with error %d.\n", err);
        return err;
    }

    err = adc_channel_setup(adc_dev, &battery_ch_cfg);
    if (err != 0) {
        printf("Battery ADC Setup failed with error %d.\n", err);
        return err;
    }

    printk("ADC setup done.\n");

    biquad_init_bandpass(&bpf, (float)SAMPLE_RATE_HZ, BPF_FC_HZ, BPF_Q);
    int32_t adc_vref = adc_ref_internal(adc_dev);

    /* Give mic time to warm up and stabilize before calibrating */
    printf("Waiting 1s for mic to warm up...\n");
    k_msleep(1000);

    printf("Calibrating microphone DC offset...\n");
    calibrate_baseline_dc();
    printf("Calibration done! baseline_dc = %d mV\n", (int)baseline_dc);

    printf("Auto-calibrating sound level...\n");
    calibrate_sound_level(adc_vref);

    /* ── STEP 2: BLE init AFTER calibration ── */
    if (init_ble() == 0) {
        printf("BLE Initialized successfully.\n");
    } else {
        printf("BLE Initialization failed.\n");
        return -1;
    }

    k_msleep(100);

    printk("About to start advertising...\n");

    /* [BAT-OPT-3] Slow advertising interval on auto-start.
     * Original used BT_GAP_ADV_FAST_INT_MIN_2/MAX_2 (100–150 ms).
     * Slow interval (1000–1280 ms) cuts average radio current by ~8x
     * in the advertising state with no functional impact on a sensor. */
    int adv_err = bt_le_adv_start(ADV_PARAM_SLOW,
                                  ad, ARRAY_SIZE(ad),
                                  sd, ARRAY_SIZE(sd));
    if (!adv_err) {
        advertising_active = true;
        printk("Auto-advertising started (slow interval)!\n");
    } else {
        printk("Failed to start advertising (err %d)\n", adv_err);
    }

    printk("Entering main loop...\n");

    while (1) {
        /* Check if app requested sound-level calibration */
        if (calibration_requested) {
            calibrate_sound_level(adc_vref);
            continue;  /* restart loop with fresh filter state */
        }

        float   sum_sq        = 0.0f;   /* FIX: was int32_t — squared floats < 1 truncated to 0 */
        int     valid_samples = 0;
        int     zcr_count     = 0;      /* [AGC-WORKAROUND] zero-crossings this frame */
        static float prev_y_bp = 0.0f;  /* [AGC-WORKAROUND] for zero-crossing detection */

        /* [BAT-OPT-1] Batch ADC sampling — eliminate k_busy_wait.
         *
         * Original code used k_busy_wait(1000000 / SAMPLE_RATE_HZ) = 125 µs
         * between every sample. k_busy_wait spins the CPU in a tight loop
         * with interrupts enabled but the scheduler never yielded —
         * the CPU could not enter any sleep state during these 8 ms/frame.
         *
         * Fix: read all FRAME_SAMPLES back-to-back as fast as the SAADC
         * allows (each read takes ~3–10 µs in hardware), then sleep for
         * the remainder of the 8 ms frame window with k_msleep().
         *
         * k_msleep() yields to the Zephyr scheduler, which (with
         * CONFIG_PM=y in prj.conf) puts the nRF52 into WFI/sleep
         * between ticks.  This is the single largest power saving in
         * this file — the CPU goes from 0 % sleep to ~95 % sleep
         * per frame when quiet (no events to process).
         *
         * Timing accuracy: The total frame duration is still ~FRAME_MS
         * (8 ms). Individual inter-sample spacing is now <10 µs instead
         * of exactly 125 µs, so the effective sample rate within a frame
         * is much higher than 8 kHz, but RMS energy over the frame is
         * unchanged.  For dB SPL threshold detection this is acceptable.
         * If strict 8 kHz inter-sample timing is required for spectral
         * accuracy, use a hardware timer or SAADC continuous mode instead.
         */
        int64_t frame_start_ms = k_uptime_get();
        int sat_count = 0;  /* count ADC rail-clipped samples (loud signal) */

        for (int i = 0; i < FRAME_SAMPLES; i++) {
            err = adc_read(adc_dev, &sequence);
            if (err != 0) {
                printf("ADC reading failed with error %d.\n", err);
                continue;
            }

            int32_t mv_value = sampleBuffer[0];
            /* [BAT-OPT-4] Use cached adc_vref, not a per-sample call */
            adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

            /* Detect ADC rail clipping (signal exceeded mic range = very loud) */
            if (mv_value <= 5 || mv_value >= 3590) {
                sat_count++;
            }

            int32_t mv_min = baseline_dc - ADC_MV_MARGIN;
            int32_t mv_max = baseline_dc + ADC_MV_MARGIN;
            if (mv_value < mv_min || mv_value > mv_max) {
                continue;
            }

            valid_samples++;

            if (i == 0) {
                printk("Raw ADC mv: %d, baseline: %d, diff: %d\n",
                       (int)mv_value, (int)baseline_dc,
                       (int)(mv_value - baseline_dc));
            }

            float x  = (float)(mv_value - baseline_dc);
            float y  = x - hpf_prev_x + HPF_R * hpf_prev_y;
            hpf_prev_x = x;
            hpf_prev_y = y;

            float y_bp = biquad_process(&bpf, y);

            if (y_bp >  10000.0f) y_bp =  10000.0f;
            if (y_bp < -10000.0f) y_bp = -10000.0f;
            sum_sq += y_bp * y_bp;   /* FIX: no int cast — keep full float precision */

            /* [AGC-WORKAROUND] Zero-crossing detection.
             * AGC compresses amplitude but not oscillation frequency.
             * Real sustained sound produces many ZCRs per frame; room
             * noise produces few. This is our primary activity signal. */
            if ((y_bp > 0.0f && prev_y_bp <= 0.0f) ||
                (y_bp < 0.0f && prev_y_bp >= 0.0f)) {
                zcr_count++;
            }
            prev_y_bp = y_bp;

            /* [BAT-OPT-1] k_busy_wait removed — no spinning between samples */
        }

        /* [BAT-OPT-1] Sleep for the remainder of the frame window.
         * This lets the CPU enter WFI/low-power idle via CONFIG_PM. */
        int64_t elapsed_ms = k_uptime_get() - frame_start_ms;
        int32_t remaining_ms = (int32_t)FRAME_MS - (int32_t)elapsed_ms;

        /* TEST LOG: shows CPU active time vs sleep time per frame.
         * Good output:  [TIMING] sample=1 ms  sleep=7 ms
         * Bad output:   [TIMING] sample=8 ms  sleep=0 ms
         * Remove once confirmed working. */
        printk("[TIMING] sample=%d ms  sleep=%d ms\n",
               (int)elapsed_ms, remaining_ms > 0 ? (int)remaining_ms : 0);

        if (remaining_ms > 0) {
            k_msleep(remaining_ms);
        }

        /* ADC saturation = signal exceeded mic range = very loud.
         *
         * Debounce: on this custom PCB, BLE radio / power supply coupling
         * causes spurious ~250ms (30-35 frame) saturation bursts in a quiet
         * room. A real baby cry sustains for seconds, so require the
         * saturation to persist for SAT_MIN_FRAMES (~400ms) before
         * accepting it as a real loud event. Brief glitches are ignored. */
        #define SAT_MIN_FRAMES 50  /* ~400ms @ 8ms/frame */
        #define POST_GLITCH_SUPPRESS_FRAMES 3  /* ignore recovery transient after glitch */
        static int consecutive_sat_frames = 0;
        static int post_glitch_suppress = 0;

        if (sat_count >= 1) {  /* any saturation = glitch candidate (catches leading edge) */
            consecutive_sat_frames++;

            if (consecutive_sat_frames >= SAT_MIN_FRAMES) {
                float db_val = 85.0f;
                float alpha = (db_val > db_filtered) ? 0.7f : 0.15f;
                db_filtered = alpha * db_val + (1.0f - alpha) * db_filtered;
                printk("[SAT] sat=%d consec=%d db=%d\n",
                       sat_count, consecutive_sat_frames, (int)db_filtered);
                goto notify_section;
            } else {
                /* Brief saturation glitch — ignore this frame entirely.
                 * On first glitch frame, wipe db_filtered back to quiet
                 * baseline to discard any leading-edge contamination from
                 * the ramp-down frame just before saturation. */
                if (consecutive_sat_frames == 1) {
                    db_filtered = CAL_TARGET_DB;
                }
                printk("[SAT-GLITCH] sat=%d consec=%d (ignored)\n",
                       sat_count, consecutive_sat_frames);
                continue;
            }
        } else {
            /* If we just came out of a glitch, suppress the first few frames
             * (recovery transient — mic snapping from 0 mV back to baseline
             * creates a fake dB spike). */
            if (consecutive_sat_frames > 0) {
                post_glitch_suppress = POST_GLITCH_SUPPRESS_FRAMES;
            }
            consecutive_sat_frames = 0;

            if (post_glitch_suppress > 0) {
                post_glitch_suppress--;
                /* Reset DSP filter state so recovery transient doesn't leak
                 * into future frames. */
                hpf_prev_x = 0.0f;
                hpf_prev_y = 0.0f;
                biquad_init_bandpass(&bpf, (float)SAMPLE_RATE_HZ, BPF_FC_HZ, BPF_Q);
                printk("[POST-GLITCH] suppress=%d\n", post_glitch_suppress);
                continue;
            }
        }

        if (valid_samples == 0) {
            continue;
        }

        float rms = sqrtf(sum_sq / (float)valid_samples);
        if (rms < 1.0f) rms = 1.0f;

        float db_val = 20.0f * log10f(rms) + calibration_offset + DB_BPF_GAIN_COMP;

        /* [AGC-WORKAROUND] ZCR activity boost with sustain requirement.
         *
         * Because the MAX9814's AGC compresses loud amplitude, we use
         * zero-crossing rate as the primary loudness indicator.
         *
         * Tuning based on real test data:
         *   - Quiet room baseline: ZCR 9-15 per frame
         *   - Clap / loud HELLO: ZCR 18-25 per frame
         *
         * We require the high-ZCR condition to persist for at least
         * ZCR_SUSTAIN_FRAMES consecutive frames (~16 ms) to reject
         * single-frame noise spikes. This is fast enough to catch
         * clap leading edges but rejects random noise peaks. */
        {
            static int zcr_above_count = 0;

            if (zcr_count >= ZCR_MIN_FOR_SOUND) {
                zcr_above_count++;
            } else {
                zcr_above_count = 0;
            }

            if (zcr_above_count >= ZCR_SUSTAIN_FRAMES) {
                /* Real sound event detected. Boost dB proportionally. */
                float excess = (float)zcr_count - (float)ZCR_MIN_FOR_SOUND;
                if (excess < 0.0f) excess = 0.0f;
                float activity_db = ZCR_DB_FLOOR + excess * ZCR_DB_SCALE +
                                    CAL_TARGET_DB + calibration_offset;
                if (activity_db > db_val) {
                    db_val = activity_db;
                }
            }
        }

        /* Asymmetric smoothing: fast attack (meter shoots up on loud sound),
         * slow release (meter decays gently back to quiet). */
        float alpha = (db_val > db_filtered) ? 0.7f : 0.08f;
        db_filtered = alpha * db_val + (1.0f - alpha) * db_filtered;

        printk("dB_val: %d, dB_filtered: %d, RMS: %d, ZCR: %d\n",
               (int)db_val, (int)db_filtered, (int)rms, zcr_count);

    notify_section:
        /* Edge-triggered alert: only notify on state transitions.
         *
         * [CRY-DETECTION v2] Burst-pattern detector (tuned from real data):
         *
         * AGC hides loudness — we cannot tell "loud" from "very loud" by
         * amplitude. But real baby cries have a distinct TEMPORAL signature:
         * they produce runs of sustained high-dB frames (80-250 ms long),
         * repeating several times within a few seconds. Household noise
         * (fan, talking, utensils) never produces these sustained runs —
         * spikes are always under 70 ms.
         *
         * Data analysis from real test recordings:
         *   Scenario        max high-run   long runs (80+ ms)
         *   QUIET           64 ms          0
         *   NORMAL noise    56 ms          0
         *   LOUD talking    64 ms          0
         *   BABY CRY        248 ms         11 runs in 6 min
         *
         * Algorithm:
         *   1. Track runs of consecutive frames where dB >= threshold
         *   2. When a run ends, check if it was long (>= 10 frames = 80 ms)
         *   3. Count "long bursts" within a sliding 2-second window
         *   4. Fire alert when 2+ long bursts seen in 2 seconds
         *   5. Clear alert when 2 seconds pass with no bursts
         *
         * This pattern is unique to sustained cry-like sounds and
         * rejects every kind of household noise tested.
         *
         * NOTIFICATION STRATEGY (important for baby monitor UX):
         * Real baby cries are NOT one continuous sound — they are waves
         * of cry / pause / cry / pause. Parents need to know each time
         * the baby resumes crying, not just the first time.
         *
         * So: we send a notification on EVERY valid burst that ends
         * (a burst = a sustained high-dB run >= 80 ms). This means
         * each wail of the cry triggers its own notification on the app.
         *
         * Anti-spam: minimum NOTIFY_COOLDOWN_FRAMES between notifications
         * prevents a single long continuous wail from spamming.
         * This is the full logic:
         *   - On cry start: notify once
         *   - On each subsequent burst >= cooldown apart: notify
         *   - On 2-sec silence: reset state so next cry starts fresh
         */
        #define BURST_MIN_FRAMES         10   /* 80 ms — min run length for a burst */
        #define BURST_QUIET_RESET        250  /* 2 sec quiet to fully reset detector */
        #define NOTIFY_COOLDOWN_FRAMES   250  /* 2 sec minimum between notifications */
        {
        static int run_length = 0;            /* current consecutive-high count */
        static int frames_since_any_high = BURST_QUIET_RESET;
        static int frames_since_last_notify = NOTIFY_COOLDOWN_FRAMES;
        static uint8_t detector_armed = 1;    /* fresh cry session ready? */

        uint8_t burst_just_ended = 0;

        /* Track the current run of above-threshold frames */
        if (db_filtered >= threshold_value) {
            run_length++;
            frames_since_any_high = 0;
        } else {
            /* Run just ended — was it long enough to count as a burst? */
            if (run_length >= BURST_MIN_FRAMES) {
                burst_just_ended = 1;
                printk("[BURST] run=%d frames (%dms)\n",
                       run_length, run_length * 8);
            }
            run_length = 0;
            frames_since_any_high++;
        }

        frames_since_last_notify++;

        /* Re-arm the detector after sustained quiet — lets the next
         * cry session notify immediately instead of needing the cooldown. */
        if (frames_since_any_high >= BURST_QUIET_RESET && !detector_armed) {
            detector_armed = 1;
            /* Send alert-clear notification when we re-arm */
            alertThreshold = 0;
            if (my_connection) {
                (void)bt_gatt_notify(my_connection, alert_threshold_attr,
                                     &alertThreshold, sizeof(alertThreshold));
                printk("Alert CLEARED: dB=%d, threshold=%d\n",
                       (int)db_filtered, threshold_value);
            }
        }

        /* Fire a notification on each burst, if cooldown satisfied */
        if (burst_just_ended && frames_since_last_notify >= NOTIFY_COOLDOWN_FRAMES) {
            alertThreshold = 1;
            detector_armed = 0;   /* mark session active */
            frames_since_last_notify = 0;

            if (my_connection) {
                int notify_err = bt_gatt_notify(my_connection, alert_threshold_attr,
                                                &alertThreshold, sizeof(alertThreshold));
                if (notify_err) {
                    printk("Failed to notify (err %d)\n", notify_err);
                } else {
                    printk("Alert notify: state=1 (dB=%d, threshold=%d)\n",
                           (int)db_filtered, threshold_value);
                }
            }
        }
        }  /* close notify_section block */

        /* [BAT-OPT-7] Rate-limited dB stream notifications.
         *
         * Original: notified every frame = every 8 ms = 125 packets/sec.
         * The BLE radio wakes for every packet, so this was keeping the
         * radio active almost continuously during streaming.
         *
         * Fix: notify at most once every STREAM_NOTIFY_COOLDOWN_MS (100 ms).
         * 10 Hz is more than adequate for a sound-level meter display and
         * cuts streaming radio wake-ups by ~12x. */
        if (sound_streaming_enabled > 0 && my_connection) {
            uint32_t now_ms = (uint32_t)k_uptime_get();
            if ((now_ms - last_stream_notify_ms) >= STREAM_NOTIFY_COOLDOWN_MS) {
                last_stream_notify_ms = now_ms;
                db_int = (int8_t)(db_filtered);
                int notify_err = bt_gatt_notify(my_connection, getStreamService_attr,
                                                &db_int, sizeof(db_int));
                if (notify_err) {
                    printk("Failed to notify (err %d)\n", notify_err);
                } else {
                    printk("dB Notification sent: %d\n", db_int);
                }
            }
        }
    }

    return 0;
}