
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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

#define STRIP_NODE DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)
#define DELAY_TIME K_MSEC(5)

#define LED_BLINK_ADV_SLOW K_MSEC(1000)  // Slow blinking (yellow) for advertising
#define LED_BLINK_CONN_FAST K_MSEC(200) // Fast blinking (orange) for connecting
#define LED_CONNECTED_GREEN K_MSEC(0)   // Solid green for connected


static enum ble_state current_ble_state = BLE_STATE_IDLE;
static struct k_timer led_blink_timer;
static bool led_on = false;

struct led_rgb pixels[STRIP_NUM_PIXELS];
const struct device *strip = DEVICE_DT_GET(STRIP_NODE);

#define EN_PIN_NODE DT_NODELABEL(user_output_pin)
static const struct gpio_dt_spec pwr_En = GPIO_DT_SPEC_GET(EN_PIN_NODE, gpios);

#define PAIR_PIN DT_NODELABEL(user_input_pin)
static const struct gpio_dt_spec pair_pin = GPIO_DT_SPEC_GET(PAIR_PIN, gpios);

static struct gpio_callback input_cb_data;
static struct k_work button_work;

bool status = false;
int count = 0;
static bool advertising_active = false;
int16_t audio_buffer[AUDIO_BUFFER_SIZE];
volatile uint32_t audio_write_index = 0;

#define SLEEP_TIME_MS 100
float db = 0.0;
int16_t db_int = 0;
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 12
#define ADC_CHANNEL 0
#define ADC_PORT SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN0
#define ADC_REFERENCE ADC_REF_INTERNAL             // 0.6V
#define ADC_GAIN ADC_GAIN_1_6                      // ADC_REFERENCE * 5

#define BATTERY_ADC_CHANNEL 1                              // Battery
#define BATTERY_ADC_PORT SAADC_CH_PSELP_PSELP_AnalogInput1 // AIN1

struct adc_channel_cfg chl0_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = ADC_PORT
#endif
};

// Config for the battery ADC
struct adc_channel_cfg battery_ch_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = BATTERY_ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = BATTERY_ADC_PORT
#endif
};

int16_t sampleBuffer[1];
struct adc_sequence sequence = {
    .channels    = BIT(ADC_CHANNEL),
    .buffer      = sampleBuffer,
    .buffer_size = sizeof(sampleBuffer),
    .resolution  = ADC_RESOLUTION
};

// ADC Battery Sequence
int16_t battery_sample[1];
struct adc_sequence battery_sequence = {
    .channels    = BIT(BATTERY_ADC_CHANNEL),
    .buffer      = battery_sample,
    .buffer_size = sizeof(battery_sample),
    .resolution  = ADC_RESOLUTION
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SET_THRESHOLD_SERVICE_VAL),
};

static struct k_timer button_timer;
static bool long_press_detected = false;


// ─────────────────────────────────────────────────────────────────────────────
// ADC / SAMPLING CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────
#define NUM_SAMPLES      5
#define SAMPLE_RATE_HZ   8000
#define FRAME_SAMPLES    64
#define HPF_R            0.995f   // ~16 Hz high-pass cutoff @ 8 kHz
#define FRAME_MS         ((1000 * FRAME_SAMPLES) / SAMPLE_RATE_HZ)  // 8 ms

// Band-pass filter centred on baby-cry fundamental (~800 Hz)
#define BPF_FC_HZ        800.0f
#define BPF_Q            0.707f

// ─────────────────────────────────────────────────────────────────────────────
// dB CALIBRATION  (AGC **disabled**)
// ─────────────────────────────────────────────────────────────────────────────
//
// MEASUREMENT REFERENCE:
//   UT353 reading  : 46 – 48 dBA  (≈ 47 dB average)
//   Board reading  : 38 – 41 dB   (≈ 39 – 40 dB, filtered int)
//   Raw dB offset  : UT353 − board ≈ +7 dB
//
// HOW THE dB IS COMPUTED:
//   db = 20 * log10(rms_mV) + DB_TOTAL_OFFSET
//
//   DB_TOTAL_OFFSET replaces the old separate calibration_offset + DB_BPF_GAIN_COMP
//   pair that was tuned for AGC-on.  With AGC disabled the signal chain is
//   linear so a single scalar offset is the cleanest approach.
//
// DERIVATION:
//   Old offset  = calibration_offset(0) + DB_BPF_GAIN_COMP(25) = 25 dB
//   Measured gap = +7 dB (board reads 7 dB too low vs. UT353)
//   New offset  = 25 + 7 = 32 dB
//
// If after flashing you still see a consistent offset, adjust DB_TOTAL_OFFSET
// by the difference:
//   board too LOW  by N dB  →  increase DB_TOTAL_OFFSET by N
//   board too HIGH by N dB  →  decrease DB_TOTAL_OFFSET by N
//
#define DB_TOTAL_OFFSET  32.0f

// EMA smoothing (α = 0.1 → ~10-frame time constant ≈ 80 ms)
#define SMOOTHING_ALPHA  0.1f

// ─────────────────────────────────────────────────────────────────────────────
// SOUND LEVEL THRESHOLDS  (kept in real-world dB SPL now that we're calibrated)
// ─────────────────────────────────────────────────────────────────────────────
#define QUIET_THRESHOLD_MAX   50.0f   // < 50 dB  → Quiet
#define MEDIUM_THRESHOLD_MAX  70.0f   // 50–70 dB → Medium
// > 70 dB → Loud

static float db_filtered = 0.0f;
static int32_t baseline_dc = 1500;

typedef struct {
    float a0, a1, a2, b1, b2, z1, z2;
} biquad_t;

static biquad_t bpf;


static float hpf_prev_x = 0.0f;
static float hpf_prev_y = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// dB ALERT GATING
// ─────────────────────────────────────────────────────────────────────────────
#define TRIGGER_HOLD_MS    200
#define RESET_HOLD_MS      100
#define NOTIFY_COOLDOWN_MS 400

static uint32_t above_ms   = 0;
static uint32_t below_ms   = 0;
static uint32_t last_notify_ms = 0;
static int notify_armed    = 1;


// ─────────────────────────────────────────────────────────────────────────────
// BABY CRY DETECTION  (raw dB burst-counting state machine)
// ─────────────────────────────────────────────────────────────────────────────
//
// Now that the dB values are calibrated to real SPL, these thresholds are
// expressed in actual dB SPL.
//
// Typical baby cry: 60 – 80 dB SPL at 1 m.
// Quiet room (baby sleeping): 40 – 50 dB SPL.
//
// Entry threshold is set conservatively low (55 dB) to catch the rising edge
// of a cry before it reaches full volume, while the lift guard (+6 dB above
// the rolling baseline) prevents false triggers from slow ambient rises.
//
#define CRY_RAW_ENTER_DB          55.0f   // SPL: cry onset (dataset: spikes to 60–71)
#define CRY_RAW_EXIT_DB           52.0f   // SPL: inter-burst dip / back to baseline
#define CRY_BURST_MIN_MS          300     // Reject spikes < 300 ms (cough, door)
#define CRY_BURST_MAX_MS          12000   // Reject continuous sounds > 12 s (TV, fan)
#define CRY_BURST_COUNT_REQUIRED  2       // Minimum bursts in window to confirm cry
#define CRY_EPISODE_WINDOW_MS     15000   // Rolling window for burst counting (15 s)
#define CRY_RISE_GUARD_DB         6.0f    // Must lift ≥ 6 dB above rolling baseline
#define CRY_ALERT_COOLDOWN_MS     5000    // 5 s between BLE notifications
#define BASELINE_EMA_ALPHA        0.05f   // Slow baseline tracker (quiet periods only)
#define BURST_HISTORY_SIZE        8

typedef enum {
    CRY_STATE_IDLE = 0,
    CRY_STATE_BURST_ACTIVE,
    CRY_STATE_CONFIRMED,
} cry_sm_state_t;

typedef struct {
    cry_sm_state_t  state;
    uint32_t        burst_start_ms;
    float           burst_peak_raw_db;
    uint32_t        burst_end_times_ms[BURST_HISTORY_SIZE];
    uint8_t         burst_head;
    uint8_t         burst_count_total;
    float           baseline_raw_db;
    uint32_t        last_alert_ms;
    bool            cry_detected;
    bool            episode_active;
    uint8_t         confirmed_burst_count;
} baby_cry_sm_t;

static baby_cry_sm_t cry_sm = {
    .state              = CRY_STATE_IDLE,
    .baseline_raw_db    = 47.0f,   // initialise near expected quiet-room SPL
    .last_alert_ms      = 0,
    .cry_detected       = false,
    .episode_active     = false,
};

// ─────────────────────────────────────────────────────────────────────────────
// DC BASELINE CALIBRATION
// ─────────────────────────────────────────────────────────────────────────────
#define NUM_CAL_SAMPLES 5

static void button_work_handler(struct k_work *work)
{
    if (k_timer_status_get(&button_timer) > 0) {
        // Timer expired before release → long press
        printf("Long press detected!\n");
        gpio_pin_set_dt(&pwr_En, 0);
        while (1) {
            k_sleep(K_FOREVER);
        }
    } else {
        // Released before timer expired → short press
        printf("Short press detected. Toggling BLE advertising...\n");
        int err;
        if (advertising_active) {
            err = bt_le_adv_stop();
            if (!err) {
                advertising_active = false;
                update_led_state(BLE_STATE_IDLE);
            }
        } else {
            err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
            if (!err) {
                advertising_active = true;
                update_led_state(BLE_STATE_ADVERTISING);
            }
        }
    }

    k_timer_stop(&button_timer); // cleanup
}

void button_timer_expiry(struct k_timer *timer_id)
{
    long_press_detected = true;
    k_work_submit(&button_work); // Submit the work handler for long press
}

void input_pin_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    bool pin_state = gpio_pin_get_dt(&pair_pin);

    if (pin_state) {
        // button pressed → start timer
        k_timer_start(&button_timer, K_SECONDS(3), K_NO_WAIT);
    } else {
        // button released → schedule work to decide short/long
        k_work_submit(&button_work);
    }
}

void update_led_strip(uint8_t r, uint8_t g, uint8_t b)
{
    // Set the RGB values for all the pixels
    for (int i = 0; i < STRIP_NUM_PIXELS; i++)
    {
        pixels[i].r = r;
        pixels[i].g = g;
        pixels[i].b = b;
    }

    // Update the LED strip
    led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
}

void update_led_state(enum ble_state state)
{
    current_ble_state = state;

    switch (state)
    {
    case BLE_STATE_ADVERTISING:
         update_led_strip(255, 255, 0); // Yellow
        break;

    case BLE_STATE_CONNECTING:
      update_led_strip(255, 165, 0); // Orange
        break;

    case BLE_STATE_CONNECTED:
        // k_timer_stop(&led_blink_timer);
        update_led_strip(0, 255, 0); // Solid green
        break;

    default:
        // k_timer_stop(&led_blink_timer);
        update_led_strip(255, 0, 0); // Solid red (idle/error)
        break;
    }
}

static void biquad_init_bandpass(biquad_t *s, float fs, float f0, float q)
{
    float w0     = 2.0f * (float)M_PI * f0 / fs;
    float sin_w0 = sinf(w0), cos_w0 = cosf(w0);
    float alpha  = sin_w0 / (2.0f * q);
    float b0 = q * alpha, b1 = 0.0f, b2 = -q * alpha;
    float a0 = 1.0f + alpha, a1 = -2.0f * cos_w0, a2 = 1.0f - alpha;
    s->a0 = b0/a0; s->a1 = b1/a0; s->a2 = b2/a0;
    s->b1 = a1/a0; s->b2 = a2/a0;
    s->z1 = 0.0f;  s->z2 = 0.0f;
}

static inline float biquad_process(biquad_t *s, float x)
{
    float y = s->a0 * x + s->z1;
    s->z1   = s->a1 * x - s->b1 * y + s->z2;
    s->z2   = s->a2 * x - s->b2 * y;
    return y;
}

static const char *get_sound_level_string(float db_value)
{
    if (db_value < QUIET_THRESHOLD_MAX)  return "Quiet";
    if (db_value < MEDIUM_THRESHOLD_MAX) return "Medium";
    return "Loud";
}

static uint8_t count_recent_bursts(uint32_t now_ms)
{
    uint8_t n    = 0;
    uint8_t size = (cry_sm.burst_count_total < BURST_HISTORY_SIZE)
                   ? cry_sm.burst_count_total : BURST_HISTORY_SIZE;
    for (uint8_t i = 0; i < size; i++) {
        if ((now_ms - cry_sm.burst_end_times_ms[i]) <= CRY_EPISODE_WINDOW_MS) n++;
    }
    return n;
}

static void record_burst(uint32_t end_ms)
{
    cry_sm.burst_end_times_ms[cry_sm.burst_head] = end_ms;
    cry_sm.burst_head = (cry_sm.burst_head + 1) % BURST_HISTORY_SIZE;
    cry_sm.burst_count_total++;
}

void update_cry_detector_raw(float raw_db, uint32_t now_ms)
{
    cry_sm.cry_detected = false;

    // Update rolling baseline only during quiet frames
    if (raw_db < CRY_RAW_EXIT_DB) {
        cry_sm.baseline_raw_db = BASELINE_EMA_ALPHA * raw_db
                                 + (1.0f - BASELINE_EMA_ALPHA) * cry_sm.baseline_raw_db;
    }

    float lift = raw_db - cry_sm.baseline_raw_db;

    switch (cry_sm.state) {

    case CRY_STATE_IDLE:
        if (raw_db >= CRY_RAW_ENTER_DB && lift >= CRY_RISE_GUARD_DB) {
            cry_sm.burst_start_ms    = now_ms;
            cry_sm.burst_peak_raw_db = raw_db;
            cry_sm.state             = CRY_STATE_BURST_ACTIVE;
            printf("CRY: [BURST START] raw=%.1f dB, lift=%.1f dB\n", raw_db, lift);
        }
        break;

    case CRY_STATE_BURST_ACTIVE:
        if (raw_db > cry_sm.burst_peak_raw_db) cry_sm.burst_peak_raw_db = raw_db;
        {
            uint32_t dur = now_ms - cry_sm.burst_start_ms;

            if (raw_db < CRY_RAW_EXIT_DB) {
                if (dur < CRY_BURST_MIN_MS) {
                    printf("CRY: [REJECT SHORT] %u ms, peak=%.1f dB\n",
                           dur, cry_sm.burst_peak_raw_db);
                    cry_sm.state = CRY_STATE_IDLE;
                    break;
                }
                if (dur > CRY_BURST_MAX_MS) {
                    printf("CRY: [REJECT LONG] %u ms\n", dur);
                    cry_sm.state = CRY_STATE_IDLE;
                    break;
                }
                record_burst(now_ms);
                uint8_t recent = count_recent_bursts(now_ms);
                printf("CRY: [VALID BURST] %u ms, peak=%.1f dB, bursts=%u/%u\n",
                       dur, cry_sm.burst_peak_raw_db, recent, CRY_BURST_COUNT_REQUIRED);
                cry_sm.state = CRY_STATE_IDLE;

                if (recent >= CRY_BURST_COUNT_REQUIRED) {
                    uint32_t elapsed = now_ms - cry_sm.last_alert_ms;
                    if (elapsed >= CRY_ALERT_COOLDOWN_MS || cry_sm.last_alert_ms == 0) {
                        cry_sm.cry_detected          = true;
                        cry_sm.episode_active        = true;
                        cry_sm.confirmed_burst_count = recent;
                        cry_sm.last_alert_ms         = now_ms;
                        cry_sm.state                 = CRY_STATE_CONFIRMED;
                        printf("CRY: *** BABY CRY CONFIRMED *** %u bursts\n", recent);
                    }
                }
            } else if (dur > CRY_BURST_MAX_MS) {
                printf("CRY: [REJECT LONG - ongoing] %u ms\n", dur);
                cry_sm.state = CRY_STATE_IDLE;
            }
        }
        break;

    case CRY_STATE_CONFIRMED:
        cry_sm.episode_active = true;
        if ((now_ms - cry_sm.last_alert_ms) >= CRY_ALERT_COOLDOWN_MS) {
            cry_sm.episode_active = false;
            cry_sm.state          = CRY_STATE_IDLE;
        }
        break;

    default:
        cry_sm.state = CRY_STATE_IDLE;
        break;
    }
}

static inline bool    is_baby_cry_detected(void)  { return cry_sm.cry_detected; }
static inline bool    is_cry_episode_active(void) { return cry_sm.episode_active; }
static inline uint8_t get_cry_burst_count(void)   { return cry_sm.confirmed_burst_count; }

static void reset_cry_detection(void)
{
    cry_sm.state                 = CRY_STATE_IDLE;
    cry_sm.cry_detected          = false;
    cry_sm.episode_active        = false;
    cry_sm.confirmed_burst_count = 0;
    cry_sm.burst_count_total     = 0;
    cry_sm.burst_head            = 0;
    memset(cry_sm.burst_end_times_ms, 0, sizeof(cry_sm.burst_end_times_ms));
    printf("CRY: Detection reset\n");
}

void calibrate_baseline_dc(void)
{
    int32_t total = 0;
    for (int i = 0; i < NUM_CAL_SAMPLES; i++) {
        if (adc_read(adc_dev, &sequence) == 0) {
            int32_t mv = sampleBuffer[0];
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &mv);
            total += mv;
        }
        k_msleep(5);
    }
    baseline_dc = total / NUM_CAL_SAMPLES;
    printf("Calibrated baseline DC: %d mV\n", baseline_dc);
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────────────────────
int main(void)
{
    int err;
    printf("Startup\n");
    update_led_strip(0, 255, 255);

    k_work_init(&button_work, button_work_handler);
    k_timer_init(&button_timer, button_timer_expiry, NULL);
    update_led_state(BLE_STATE_IDLE);

    if (!device_is_ready(pwr_En.port)) { printf("GPIO port not ready\n"); return 0; }
    gpio_pin_configure_dt(&pwr_En, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&pwr_En, 1);

    if (!gpio_is_ready_dt(&pair_pin)) return 0;
    gpio_pin_configure_dt(&pair_pin, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&pair_pin, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&input_cb_data, input_pin_isr, BIT(pair_pin.pin));
    gpio_add_callback(pair_pin.port, &input_cb_data);

    if (init_ble() == 0) printf("BLE Initialized successfully.\n");
    else                  printf("BLE Initialization failed.\n");

    advertising_active = false;
    update_led_strip(255, 0, 0);

    if (!device_is_ready(adc_dev)) { printf("ADC Device not ready\n"); return 0; }

    err = adc_channel_setup(adc_dev, &chl0_cfg);
    if (err) { printf("ADC Setup failed: %d\n", err); return 0; }

    err = adc_channel_setup(adc_dev, &battery_ch_cfg);
    if (err) { printf("Battery ADC Setup failed: %d\n", err); return 0; }

    biquad_init_bandpass(&bpf, (float)SAMPLE_RATE_HZ, BPF_FC_HZ, BPF_Q);
    audio_init();

    printf("Calibrating microphone DC offset...\n");
    calibrate_baseline_dc();

    while (1)
    {
        // ── Dedicated audio recording loop (cycle-accurate timing) ─────
        if (audio_recording) {
            printf("Audio: Recording at %d Hz...\n", SAMPLE_RATE_HZ);
            uint32_t cycles_per_sample = sys_clock_hw_cycles_per_sec() / SAMPLE_RATE_HZ;
            uint32_t next_cycle = k_cycle_get_32();

            while (audio_recording && audio_write_index < AUDIO_BUFFER_SIZE) {
                next_cycle += cycles_per_sample;

                err = adc_read(adc_dev, &sequence);
                if (err == 0) {
                    int32_t mv_value = sampleBuffer[0];
                    adc_raw_to_millivolts(adc_ref_internal(adc_dev),
                                          ADC_GAIN, ADC_RESOLUTION, &mv_value);
                    audio_buffer[audio_write_index++] = (int16_t)mv_value;
                }

                // Spin until exact next sample time
                while ((int32_t)(next_cycle - k_cycle_get_32()) > 0) { }
            }
            audio_recording = false;
            audio_status = AUDIO_STATUS_IDLE;
            printf("Audio: Recording done (%u samples)\n", audio_write_index);
            continue;
        }

        // ── Live audio streaming over BLE ────────────────────────────────
        if (audio_stream_active && my_connection) {
            uint16_t mtu = bt_gatt_get_mtu(my_connection);
            if (mtu < 23) mtu = 23;
            uint16_t max_payload = mtu - 3;     // ATT notification header
            uint16_t frame_size = max_payload - 2; // 2-byte seq header
            if (frame_size > 240) frame_size = 240;

            uint8_t frame[244];
            static uint16_t stream_seq = 0;
            frame[0] = (uint8_t)(stream_seq & 0xFF);
            frame[1] = (uint8_t)(stream_seq >> 8);
            stream_seq++;

            // Sample one frame of 8-bit audio using same timing as recording
            uint32_t cycles_per_sample = sys_clock_hw_cycles_per_sec() / SAMPLE_RATE_HZ;
            uint32_t next_cycle = k_cycle_get_32();

            for (uint16_t i = 0; i < frame_size; i++) {
                next_cycle += cycles_per_sample;
                err = adc_read(adc_dev, &sequence);
                if (err == 0) {
                    int16_t raw = sampleBuffer[0];
                    if (raw < 0) raw = 0;
                    frame[2 + i] = (uint8_t)((raw >> 4) & 0xFF);
                } else {
                    frame[2 + i] = 128;
                }
                while ((int32_t)(next_cycle - k_cycle_get_32()) > 0) { }
            }

            bt_gatt_notify(my_connection, audio_data_attr, frame, frame_size + 2);
            continue;
        }

        // ── Sample one frame ──────────────────────────────────────────────
        int32_t sum_sq = 0;
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            err = adc_read(adc_dev, &sequence);
            if (err != 0) { printf("ADC read error %d\n", err); continue; }

            int32_t mv_value = sampleBuffer[0];
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &mv_value);

            float x  = (float)(mv_value - baseline_dc);

            // DC-blocking high-pass filter
            float y  = x - hpf_prev_x + HPF_R * hpf_prev_y;
            hpf_prev_x = x;
            hpf_prev_y = y;

            // Band-pass filter (emphasises 800 Hz region)
            float y_bp = biquad_process(&bpf, y);

            // Clamp to avoid int32 overflow in accumulator
            if (y_bp >  3000.0f) y_bp =  3000.0f;
            if (y_bp < -3000.0f) y_bp = -3000.0f;
            sum_sq += (int32_t)(y_bp * y_bp);

            k_busy_wait(1000000 / SAMPLE_RATE_HZ);   // pacing for 8 kHz
        }

        // ── Compute dB SPL ────────────────────────────────────────────────
        float rms = sqrtf((float)sum_sq / FRAME_SAMPLES);
        if (rms < 1.0f) rms = 1.0f;   // avoid log10(0)

        // DB_TOTAL_OFFSET = BPF gain compensation (25 dB) + calibration vs UT353 (+7 dB)
        db          = 20.0f * log10f(rms) + DB_TOTAL_OFFSET;
        db_filtered = SMOOTHING_ALPHA * db + (1.0f - SMOOTHING_ALPHA) * db_filtered;
        db_int      = (int8_t)(db_filtered);

        printf("Sound Level: %s (dB=%d)\n", get_sound_level_string(db_filtered), db_int);
        k_msleep(25);

        // ── Baby cry detection ────────────────────────────────────────────
        uint32_t now_ms = k_uptime_get_32();
        update_cry_detector_raw(db, now_ms);   // pass RAW db (unfiltered)

        if (is_baby_cry_detected()) {
            baby_cry_detected = 1;

            if (my_connection) {
                err = bt_gatt_notify(my_connection, baby_cry_attr,
                                     &baby_cry_detected, sizeof(baby_cry_detected));
                if (err) printf("Failed to notify baby cry (err %d)\n", err);
                else     printf("Baby Cry Notification sent: %d\n", baby_cry_detected);}
        } else if (!is_cry_episode_active()) {
            // Reset baby cry flag when episode ends
            baby_cry_detected = 0;
        }

        // ── Threshold alert ───────────────────────────────────────────────
        if (db_filtered >= threshold_value) {
            above_ms += FRAME_MS;
            below_ms  = 0;
            alertThreshold = 1;

            if (my_connection) {
                err = bt_gatt_notify(my_connection, alert_threshold_attr,
                                     &alertThreshold, sizeof(alertThreshold));
                if (err) printf("Failed to notify threshold (err %d)\n", err);
            }
        } else {
            below_ms     += FRAME_MS;
            above_ms      = 0;
            alertThreshold = 0;
            if (!notify_armed && below_ms >= RESET_HOLD_MS) notify_armed = 1;
        }

        // ── dB streaming ─────────────────────────────────────────────────
        if (sound_streaming_enabled > 0 && my_connection) {
            db_int = (int8_t)(db_filtered);
            err = bt_gatt_notify(my_connection, getStreamService_attr, &db_int, sizeof(db_int));
            if (err) printf("Failed to stream dB (err %d)\n", err);
            else     printf("dB Notification sent: %d\n", db_int);
        }
    }

    return 0;
}