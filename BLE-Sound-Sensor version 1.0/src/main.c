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

/* LED strip: hardware not present on nrf52840dk, use dummy */
#define DELAY_TIME K_MSEC(5)
#define STRIP_NUM_PIXELS 1
static struct led_rgb pixels[STRIP_NUM_PIXELS];
static const struct device *strip = NULL;

#define LED_BLINK_ADV_SLOW  K_MSEC(1000)
#define LED_BLINK_CONN_FAST K_MSEC(200)
#define LED_CONNECTED_GREEN K_MSEC(0)

static enum ble_state current_ble_state = BLE_STATE_IDLE;
static struct k_timer led_blink_timer;
static bool led_on = false;

/* GPIO: use zephyr,user node properties */
#define EN_PIN_NODE   DT_PATH(zephyr_user)
#define PAIR_PIN_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec pwr_En   = GPIO_DT_SPEC_GET(EN_PIN_NODE,   user_output_gpios);
static const struct gpio_dt_spec pair_pin = GPIO_DT_SPEC_GET(PAIR_PIN_NODE, user_input_gpios);

static struct gpio_callback input_cb_data;
static struct k_work button_work;

bool status = false;
int count = 0;
static bool advertising_active = false;
#define AUDIO_BUFFER_SIZE 16000
int16_t audio_buffer[AUDIO_BUFFER_SIZE];
volatile uint32_t audio_write_index = 0;

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

/* Advertising data: flags + device name */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Scan response data: UUID */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SET_THRESHOLD_SERVICE_VAL),
};

static struct k_timer button_timer;
static bool long_press_detected = false;

static void button_work_handler(struct k_work *work)
{
    if (k_timer_status_get(&button_timer) > 0) {
        printk("Long press detected!\n");
        gpio_pin_set_dt(&pwr_En, 0);
        while (1) {
            k_sleep(K_FOREVER);
        }
    } else {
        printk("Short press detected. Toggling BLE advertising...\n");
        int err;
        if (advertising_active) {
            err = bt_le_adv_stop();
            if (!err) {
                advertising_active = false;
                update_led_state(BLE_STATE_IDLE);
            }
        } else {
            err = bt_le_adv_start(
                BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
                    BT_GAP_ADV_FAST_INT_MIN_2,
                    BT_GAP_ADV_FAST_INT_MAX_2,
                    NULL),
                ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
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
}

#define NUM_SAMPLES       5
#define SMOOTHING_ALPHA   0.1f
#define DEFAULT_DB_OFFSET 0.0f

static float   db_filtered        = 0.0f;
static float   calibration_offset = DEFAULT_DB_OFFSET;
static int32_t baseline_dc        = 1500;

#define SAMPLE_RATE_HZ   8000
#define FRAME_SAMPLES    64
#define HPF_R            0.995f
#define FRAME_MS         ((1000 * FRAME_SAMPLES) / SAMPLE_RATE_HZ)

#define BPF_FC_HZ        2000.0f  /* Widened from 800Hz to catch more sound */
#define BPF_Q            0.3f     /* Wider bandwidth */
#define DB_BPF_GAIN_COMP 20.0f    /* Increased sensitivity */

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

void calibrate_baseline_dc(void)
{
    int32_t total = 0;
    int err;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        err = adc_read(adc_dev, &sequence);
        if (err == 0) {
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

int main(void)
{
    int err;
    printk("Startup\n");

    k_work_init(&button_work, button_work_handler);
    k_timer_init(&button_timer, button_timer_expiry, NULL);

    update_led_state(BLE_STATE_IDLE);

    /* Restore power enable pin — keeps board powered on */
    if (!device_is_ready(pwr_En.port)) {
        printk("GPIO port not ready\n");
    } else {
        gpio_pin_configure_dt(&pwr_En, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&pwr_En, 1);
        printk("Power enable pin set HIGH\n");
    }

    /* Button (pair_pin) still bypassed for now */
    printk("Button bypassed for testing\n");

    if (init_ble() == 0) {
        printf("BLE Initialized successfully.\n");
    } else {
        printf("BLE Initialization failed.\n");
        return -1;
    }

    /* Small delay to let BLE stack settle */
    k_msleep(100);

    printk("About to start advertising...\n");

    int adv_err = bt_le_adv_start(
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
            BT_GAP_ADV_FAST_INT_MIN_2,
            BT_GAP_ADV_FAST_INT_MAX_2,
            NULL),
        ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (!adv_err) {
        advertising_active = true;
        printk("Auto-advertising started!\n");
    } else {
        printk("Failed to start advertising (err %d)\n", adv_err);
    }

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

    printk("ADC setup done. Calibrating microphone...\n");

    /* Calibrate microphone DC offset on startup */
    printf("Calibrating microphone DC offset...\n");
    calibrate_baseline_dc();
    printf("Calibration done!\n");

    printk("Entering main loop...\n");

    biquad_init_bandpass(&bpf, (float)SAMPLE_RATE_HZ, BPF_FC_HZ, BPF_Q);

    while (1) {
        int32_t sum_sq = 0;

        for (int i = 0; i < FRAME_SAMPLES; i++) {
            err = adc_read(adc_dev, &sequence);
            if (err != 0) {
                printf("ADC reading failed with error %d.\n", err);
                continue;
            }

            int32_t mv_value = sampleBuffer[0];
            int32_t adc_vref = adc_ref_internal(adc_dev);
            adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

            /* Debug: print raw ADC value every frame (first sample only) */
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
            sum_sq += (int32_t)(y_bp * y_bp);

            k_busy_wait(1000000 / SAMPLE_RATE_HZ);
        }

        float rms = sqrtf((float)sum_sq / FRAME_SAMPLES);
        if (rms < 1.0f) rms = 1.0f;

        float db_val = 20.0f * log10f(rms) + calibration_offset + DB_BPF_GAIN_COMP;
        db_filtered  = SMOOTHING_ALPHA * db_val + (1.0f - SMOOTHING_ALPHA) * db_filtered;

        /* Debug: print dB value every frame */
        printk("dB_val: %d, dB_filtered: %d, RMS: %d\n",
               (int)db_val, (int)db_filtered, (int)rms);

        if (db_filtered >= threshold_value) {
            above_ms += FRAME_MS;
            below_ms  = 0;

            alertThreshold = 1;
            printk("------>>>>>>>>>Value of threshold variable: %d\n", threshold_value);

            if (my_connection) {
                int notify_err = bt_gatt_notify(my_connection, alert_threshold_attr,
                                                &alertThreshold, sizeof(alertThreshold));
                if (notify_err) {
                    printk("Failed to notify (err %d)\n", notify_err);
                } else {
                    printk("Threshold notification sent: %d (dB=%.1f)\n",
                           alertThreshold, (double)db_filtered);
                }
            }
        } else {
            below_ms += FRAME_MS;
            above_ms  = 0;
            alertThreshold = 0;

            if (!notify_armed && below_ms >= RESET_HOLD_MS) {
                notify_armed = 1;
            }
        }

        if (sound_streaming_enabled > 0 && my_connection) {
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

    return 0;
}