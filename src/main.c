
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
#define AUDIO_BUFFER_SIZE 16000 // e.g., 1 second at 16 kHz
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
    .channels = BIT(ADC_CHANNEL),
    .buffer = sampleBuffer,
    .buffer_size = sizeof(sampleBuffer),
    .resolution = ADC_RESOLUTION};

// ADC Battery Sequence
int16_t battery_sample[1];

struct adc_sequence battery_sequence = {
    .channels = BIT(BATTERY_ADC_CHANNEL),
    .buffer = battery_sample,
    .buffer_size = sizeof(battery_sample),
    .resolution = ADC_RESOLUTION};

static const struct bt_data ad[] =
    {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SET_THRESHOLD_SERVICE_VAL),
};

static struct k_timer button_timer;
static bool long_press_detected = false;

static void button_work_handler(struct k_work *work)
{
    if (k_timer_status_get(&button_timer) > 0) {
        // Timer expired before release → long press
        printk("Long press detected!\n");
        gpio_pin_set_dt(&pwr_En, 0);
        while (1) {
            k_sleep(K_FOREVER);
        }
    } else {
        // Released before timer expired → short press
        printk("Short press detected. Toggling BLE advertising...\n");
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

#define NUM_SAMPLES 5
#define SMOOTHING_ALPHA 0.1f
#define DEFAULT_DB_OFFSET 0.0f

static float db_filtered = 0.0f;
static float calibration_offset = -6.0f; // Reduced from 0.0f to -6.0f to account for AGC
static int32_t baseline_dc = 1500; // This will be updated after calibration

// --- Sampling and filtering configuration ---
#define SAMPLE_RATE_HZ 8000
#define FRAME_SAMPLES 64
#define HPF_R 0.995f // ~16 Hz cutoff at 8 kHz
#define FRAME_MS ((1000 * FRAME_SAMPLES) / SAMPLE_RATE_HZ)

// --- Band-pass filter (biquad) to emphasize baby cries ---
#define BPF_FC_HZ 800.0f      // Center frequency
#define BPF_Q 0.707f          // Quality factor
// #define DB_BPF_GAIN_COMP 6.0f // dB compensation after band-pass
#define DB_BPF_GAIN_COMP 25.0f // dB compensation after band-pass

// --- Sound Level Thresholds ---
#define QUIET_THRESHOLD_MAX 40.0f         // dB - Quiet sounds (less than 40dB)
#define MEDIUM_THRESHOLD_MIN 40.0f        // dB - Moderate/Medium sounds start
#define MEDIUM_THRESHOLD_MAX 70.0f        // dB - Moderate/Medium sounds end
#define LOUD_THRESHOLD_MIN 70.0f          // dB - Loud/Dangerous sounds start (above 70dB)

typedef struct
{
    float a0;
    float a1;
    float a2;
    float b1;
    float b2;
    float z1;
    float z2;
} biquad_t;

static biquad_t bpf;

static void biquad_init_bandpass(biquad_t *s, float sample_rate_hz, float f0_hz, float q)
{
    float w0 = 2.0f * (float)M_PI * f0_hz / sample_rate_hz;
    float sin_w0 = sinf(w0);
    float cos_w0 = cosf(w0);
    float alpha = sin_w0 / (2.0f * q);

    float b0 = q * alpha; // RBJ band-pass (constant skirt gain)
    float b1 = 0.0f;
    float b2 = -q * alpha;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;

    // Normalize
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
    s->z1 = s->a1 * x - s->b1 * y + s->z2;
    s->z2 = s->a2 * x - s->b2 * y;
    return y;
}

static float hpf_prev_x = 0.0f;
static float hpf_prev_y = 0.0f;

// --- Notification gating: hold and cooldown ---
#define TRIGGER_HOLD_MS 200    // Must stay above threshold this long
#define RESET_HOLD_MS 100      // Must stay below to re-arm
#define NOTIFY_COOLDOWN_MS 400 // Minimum gap between notifications

static uint32_t above_ms = 0;
static uint32_t below_ms = 0;
static uint32_t last_notify_ms = 0;
static int notify_armed = 1;

static const char* get_sound_level_string(float db_value)
{
    if (db_value < QUIET_THRESHOLD_MAX)
    {
        return "Quiet";
    }
    else if (db_value < MEDIUM_THRESHOLD_MAX)
    {
        return "Medium";
    }
    else
    {
        return "Loud";
    }
}

void calibrate_baseline_dc(void)
{
    int32_t total = 0;
    int err;

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        err = adc_read(adc_dev, &sequence);
        if (err == 0)
        {
            int32_t mv_value = sampleBuffer[0];
            int32_t adc_vref = adc_ref_internal(adc_dev);
            adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);
            total += mv_value;
        }
        k_msleep(5);
    }

    baseline_dc = total / NUM_SAMPLES;
    printf("Calibrated baseline DC: %d mV\n", baseline_dc);
}

static bool calibration_mode = false; // Set to false after calibration

static void log_calibration_data(float rms, float db_raw)
{
    // Log raw values for comparison with UT353
    // Format: RMS | Raw dB | Filtered dB | Baseline DC
    printf("CALIB | RMS=%.2f | dB_raw=%.2f | dB_filtered=%.2f | baseline=%d mV\n",
           rms, db_raw, db_filtered, baseline_dc);
}

// --- Calibration data storage ---
#define CAL_POINTS 5
static struct {
    float reference_db;  // UT353 reading
    float sensor_db;     // Your sensor reading
} cal_data[CAL_POINTS] = {0};
static int cal_index = 0;

static void store_calibration_point(float ref_db, float sensor_db)
{
    if (cal_index < CAL_POINTS) {
        cal_data[cal_index].reference_db = ref_db;
        cal_data[cal_index].sensor_db = sensor_db;
        printf("CAL POINT %d: Reference=%.1f dB, Sensor=%.1f dB\n", 
               cal_index + 1, ref_db, sensor_db);
        cal_index++;
    }
}

static void compute_calibration_curve(void)
{
    if (cal_index < 2) {
        printf("Need at least 2 calibration points\n");
        return;
    }
    
    // Simple linear fit: sensor_db = m * reference_db + b
    float sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0;
    
    for (int i = 0; i < cal_index; i++) {
        float x = cal_data[i].reference_db;
        float y = cal_data[i].sensor_db;
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }
    
    float n = (float)cal_index;
    float m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    float b = (sum_y - m * sum_x) / n;
    
    printf("Calibration curve: sensor = %.3f * reference + %.3f\n", m, b);
    printf("Inverse: reference = (sensor - %.3f) / %.3f\n", b, m);
}

static void handle_calibration_command(const char *input)
{
    if (strncmp(input, "CAL ", 4) == 0) {
        float ref_db, sensor_db;
        int parsed = sscanf(input + 4, "%f %f", &ref_db, &sensor_db);
        
        if (parsed == 2) {
            store_calibration_point(ref_db, sensor_db);
        } else {
            printf("Invalid format. Use: CAL <reference_db> <sensor_db>\n");
            printf("Example: CAL 50.5 55.2\n");
        }
    }
    else if (strcmp(input, "COMPUTE") == 0) {
        compute_calibration_curve();
    }
    else if (strcmp(input, "SHOW") == 0) {
        printf("\nStored calibration points:\n");
        for (int i = 0; i < cal_index; i++) {
            printf("  Point %d: Reference=%.1f dB, Sensor=%.1f dB, Diff=%.1f dB\n",
                   i + 1, 
                   cal_data[i].reference_db,
                   cal_data[i].sensor_db,
                   cal_data[i].sensor_db - cal_data[i].reference_db);
        }
    }
    else if (strcmp(input, "RESET") == 0) {
        cal_index = 0;
        printf("Calibration data reset\n");
    }
}

int main(void)
{
    int err;
    printk("Startup\n");
    update_led_strip(0, 255, 255);
    /* initialize the work item (do this before gpio_add_callback) */
    k_work_init(&button_work, button_work_handler);
    // Initialize the button timer
    k_timer_init(&button_timer, button_timer_expiry, NULL);

    update_led_state(BLE_STATE_IDLE);

    if (!device_is_ready(pwr_En.port))
    {
        printk("GPIO port not ready\n");
        return;
    }
    gpio_pin_configure_dt(&pwr_En, GPIO_OUTPUT_ACTIVE); // Start HIGH (ACTIVE)
    gpio_pin_set_dt(&pwr_En, 1);                        // Set HIGH again

    // Check if the device is ready
    if (!gpio_is_ready_dt(&pair_pin))
    {
        return;
    }

    // Configure the pin as input
    gpio_pin_configure_dt(&pair_pin, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&pair_pin, GPIO_INT_EDGE_BOTH);
    /* Initialize and add the callback */
    gpio_init_callback(&input_cb_data, input_pin_isr, BIT(pair_pin.pin));
    gpio_add_callback(pair_pin.port, &input_cb_data);

    if (init_ble() == 0)
    {
        printf("BLE Initialized successfully.\n");
    }
    else
    {
        printf("BLE Initialization failed.\n");
    }
    // Start with advertising disabled - user must press button to enable
    advertising_active = false;
    update_led_strip(255, 0, 0); // Red LED to indicate not advertising

    if (!device_is_ready(adc_dev))
    {
        printf("ADC Device not ready\n");
        return;
    }

    err = adc_channel_setup(adc_dev, &chl0_cfg);
    if (err != 0)
    {
        printf("ADC Setup failed with error %d.\n", err);
        return;
    }

    // ADC Battery Setup
    err = adc_channel_setup(adc_dev, &battery_ch_cfg);
    if (err != 0)
    {
        printf("Battery ADC Setup failed with error %d.\n", err);
        return;
    }

    // Init band-pass
    biquad_init_bandpass(&bpf, (float)SAMPLE_RATE_HZ, BPF_FC_HZ, BPF_Q);

    printf("Calibrating microphone DC offset...\n");
    calibrate_baseline_dc();

    printf("\n=== CALIBRATION MODE ===\n");
    printf("Instructions:\n");
    printf("1. Make a sound at a known level\n");
    printf("2. Read the dB value from UT353 meter\n");
    printf("3. Send command: CAL <reference_db> <sensor_db>\n");
    printf("   Example: CAL 50.5 55.2\n");
    printf("4. Repeat 5 times at different sound levels (quiet to loud)\n");
    printf("5. Send: COMPUTE to calculate calibration curve\n");
    printf("======================\n\n");

    while (1)
    {
        // if (count == 1)
        // {
        //     update_led_strip(255, 0, 0);
        // }
        // if (count == 2)
        // {
        //     update_led_strip(0, 255, 0);
        // }
        // if (count == 3)
        // {
        //     update_led_strip(0, 0, 255);
        // }
        // if (count == 4)
        // {
        //     update_led_strip(255, 255, 255);
        //         gpio_pin_set_dt(&pwr_En, 0);                        // Set HIGH again

        // }
        // if (count == 5)
        // {
        //     count = 0;
        // }

        // Fixed-size frame sampling with DC blocking HPF and RMS
        int32_t sum_sq = 0;
        for (int i = 0; i < FRAME_SAMPLES; i++)
        {
            err = adc_read(adc_dev, &sequence);
            if (err != 0)
            {
                printf("ADC reading failed with error %d.\n", err);
                continue;
            }

            int32_t mv_value = sampleBuffer[0];
            int32_t adc_vref = adc_ref_internal(adc_dev);
            adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

            // Convert to centered float in mV
            float x = (float)(mv_value - baseline_dc);
            // One-pole DC blocker (high-pass)
            float y = x - hpf_prev_x + HPF_R * hpf_prev_y;
            hpf_prev_x = x;
            hpf_prev_y = y;

            // Band-pass filter
            float y_bp = biquad_process(&bpf, y);

            // RMS accumulate (limit to safe range)
            if (y_bp > 3000.0f)
                y_bp = 3000.0f;
            if (y_bp < -3000.0f)
                y_bp = -3000.0f;
            sum_sq += (int32_t)(y_bp * y_bp);

            // Sleep to approximate SAMPLE_RATE_HZ
            k_busy_wait(1000000 / SAMPLE_RATE_HZ);
        }

        float rms = sqrtf((float)sum_sq / FRAME_SAMPLES);
        if (rms < 1.0f)
            rms = 1.0f;

        float db = 20.0f * log10f(rms) + calibration_offset + DB_BPF_GAIN_COMP;
        db_filtered = SMOOTHING_ALPHA * db + (1.0f - SMOOTHING_ALPHA) * db_filtered;

        db_int = (int8_t)(db_filtered);
        // printk("Threshold notification sent:(dB=%d)\n", db_int);
        // if (calibration_mode) {
        //     log_calibration_data(rms, db);
        // }

        // Always show current sensor reading
        printf(">>> CURRENT SENSOR: %.2f dB (dB_raw=%.2f)\n", db_filtered, db);
        
        // Display current sound level status (Quiet/Medium/Loud)
        // printk("Sound Level: %s (dB=%d)\n", get_sound_level_string(db_filtered), db_int);
        k_msleep(25);

        // dB Alert Notification with hold and cooldown (one per excursion)
        uint32_t now_ms = k_uptime_get_32();

        if (db_filtered >= threshold_value)
        {
            above_ms += FRAME_MS;
            below_ms = 0;

            // if (notify_armed &&
            //     above_ms >= TRIGGER_HOLD_MS &&
            //     (now_ms - last_notify_ms) >= NOTIFY_COOLDOWN_MS)
            // {

                alertThreshold = 1;
                // printk("------>>>>>>>>>Value of threshold variable: %d\n", threshold_value);

                if (my_connection)
                {
                    int err = bt_gatt_notify(my_connection, alert_threshold_attr,
                                             &alertThreshold, sizeof(alertThreshold));
                    if (err)
                    {
                        printk("Failed to notify (err %d)\n", err);
                    }
                    else
                    {
                        // printk("Threshold notification sent: %d (dB=%.1f)\n",
                        //        alertThreshold, db_filtered);
                    }
                }
            //     last_notify_ms = now_ms;
            //     notify_armed = 0; // disarm until rearmed below
            // }
        }
        else
        {
            below_ms += FRAME_MS;
            above_ms = 0;
            alertThreshold = 0;

            if (!notify_armed && below_ms >= RESET_HOLD_MS)
            {
                notify_armed = 1; // rearm when signal low for long enough
            }
        }

        // dB Streaming Notification
        if (sound_streaming_enabled > 0 && my_connection)
        {
            db_int = (int8_t)(db_filtered);
            int err = bt_gatt_notify(my_connection, getStreamService_attr, &db_int, sizeof(db_int));
            if (err)
            {
                printk("Failed to notify (err %d)\n", err);
            }
            else
            {
                printk("dB Notification sent: %d\n", db_int);
            }
        }

        // else{
        //     sound_streaming_enabled = 0;
        // }

        // printf("Sound Level: %.2f dB\n", db);

        // --- Battery Voltage Read ---
        //  err = adc_read(adc_dev, &battery_sequence);
        //  if (err != 0) {
        //      printf("Battery ADC reading failed: %d\n", err);
        //      continue;
        //  }

        //  int32_t battery_mv = battery_sample[0];
        //  int32_t battery_vref = adc_ref_internal(adc_dev);
        //  adc_raw_to_millivolts(battery_vref, ADC_GAIN, ADC_RESOLUTION, &battery_mv);

        //  // If using a voltage divider (e.g., R1 = R2), multiply accordingly:
        //  float battery_voltage = battery_mv * 1.0f / 1000.0f; // Convert to volts
        //  printf("Battery Voltage: %.2f V\n", battery_voltage);

        // k_msleep(SLEEP_TIME_MS);
    }

    // while (1)
    // {
    //     err = adc_read(adc_dev, &sequence);
    //     if (err != 0) {
    //         continue;
    //     }

    //     // Remove DC bias (center around 0)
    //     int16_t sample = sampleBuffer[0] - 1500;

    //     // Optional: scale to 16-bit range
    //     sample <<= 4;

    //     // Print to serial for logging
    //     printf("%d\r\n", sample);

    //     // Wait to match desired sampling rate
    //     k_busy_wait(65); // ~16kHz
    // }
}
