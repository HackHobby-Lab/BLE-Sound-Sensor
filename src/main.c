// ─────────────────────────────────────────────────────────────────────────────
// Mic-Sense Audio Streaming Pipeline
//
// Capture : SAADC 12-bit → Timer2+PPI @ 16 kHz → DMA double-buffer (320 = 20 ms)
// Process : DC bias removal → 16-bit PCM → IMA ADPCM encode → 164-byte packet
// Transport: BLE Notify → web decode → 16 kHz 16-bit live playback
// ─────────────────────────────────────────────────────────────────────────────

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/irq.h>
#include <string.h>
#include <stdio.h>

#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_saadc.h>

#include "micsense_service.h"

// ─── LED strip ──────────────────────────────────────────────────────────────
#define STRIP_NODE       DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)
struct led_rgb pixels[STRIP_NUM_PIXELS];
const struct device *strip = DEVICE_DT_GET(STRIP_NODE);

// ─── GPIO ───────────────────────────────────────────────────────────────────
#define EN_PIN_NODE DT_NODELABEL(user_output_pin)
static const struct gpio_dt_spec pwr_En = GPIO_DT_SPEC_GET(EN_PIN_NODE, gpios);

#define PAIR_PIN DT_NODELABEL(user_input_pin)
static const struct gpio_dt_spec pair_pin = GPIO_DT_SPEC_GET(PAIR_PIN, gpios);

static struct gpio_callback input_cb_data;
static struct k_work button_work;
static struct k_timer button_timer;
static bool long_press_detected = false;
static bool advertising_active = false;
static enum ble_state current_ble_state = BLE_STATE_IDLE;

// ─── BLE advertising ────────────────────────────────────────────────────────
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SET_THRESHOLD_SERVICE_VAL),
};

// ─── Audio pipeline constants ───────────────────────────────────────────────
#define STREAM_RATE_HZ   16000
#define FRAME_SAMPLES    320       // 20 ms @ 16 kHz
#define ADPCM_DATA_BYTES (FRAME_SAMPLES / 2)  // 160 bytes (4 bits per sample)
#define ADPCM_HDR_BYTES  4         // [int16 predicted][uint8 step_idx][uint8 rsvd]
#define ADPCM_PKT_SIZE   (ADPCM_HDR_BYTES + ADPCM_DATA_BYTES)  // 164 bytes
#define TIMER_CC_16KHZ   1000      // 16 MHz / 1000 = 16 kHz exact

// ─── nrfx peripheral instances ──────────────────────────────────────────────
static const nrfx_timer_t sample_timer = NRFX_TIMER_INSTANCE(2);
static uint8_t ppi_channel;

// ─── DMA double buffers ────────────────────────────────────────────────────
static int16_t saadc_buf0[FRAME_SAMPLES];
static int16_t saadc_buf1[FRAME_SAMPLES];

// ─── ISR → main thread signaling ───────────────────────────────────────────
K_SEM_DEFINE(frame_sem, 0, 1);
static int16_t *volatile ready_buf = NULL;

// ─── DC bias (measured at startup from first frame) ─────────────────────────
static int16_t dc_bias = 0;
static volatile bool pipeline_running = false;
static bool dc_calibrated = false;

// ─── Globals required by micsense_service.c externs ─────────────────────────
uint8_t r = 0, g = 0, b = 0;
float db = 0.0f;
int16_t db_int = 0;
int16_t audio_buffer[AUDIO_BUFFER_SIZE];
volatile uint32_t audio_write_index = 0;

// ─── IMA ADPCM tables ──────────────────────────────────────────────────────
static const int16_t ima_step[89] = {
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,
    50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,
    230,253,279,307,337,371,408,449,494,544,598,658,724,796,
    876,963,1060,1166,1282,1411,1552,1707,1878,2066,2272,2499,
    2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,
    7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,
    20350,22385,24623,27086,29794,32767
};

static const int8_t ima_idx_adj[16] = {
    -1,-1,-1,-1, 2, 4, 6, 8,
    -1,-1,-1,-1, 2, 4, 6, 8
};

// ─── ADPCM encoder state (persistent across frames) ─────────────────────────
static int16_t enc_pred = 0;
static int8_t  enc_idx  = 0;

// ─── Forward declarations ───────────────────────────────────────────────────
void update_led_strip(uint8_t r, uint8_t g, uint8_t b);
void update_led_state(enum ble_state state);

// ─────────────────────────────────────────────────────────────────────────────
// SAADC callback (ISR context)
// ─────────────────────────────────────────────────────────────────────────────
static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
    switch (p_event->type) {
    case NRFX_SAADC_EVT_DONE:
        ready_buf = p_event->data.done.p_buffer;
        k_sem_give(&frame_sem);
        break;
    case NRFX_SAADC_EVT_BUF_REQ:
        // Re-queue the completed buffer for continuous double-buffered operation.
        // Safe: main thread processes in <5 ms, this buffer won't be reused for 20 ms.
        if (ready_buf) {
            nrfx_saadc_buffer_set((int16_t *)ready_buf, FRAME_SAMPLES);
        }
        break;
    default:
        break;
    }
}

// ─── Timer handler (unused, PPI drives sampling) ────────────────────────────
static void timer_handler(nrf_timer_event_t event, void *ctx)
{
    (void)event;
    (void)ctx;
}

// ─────────────────────────────────────────────────────────────────────────────
// Audio pipeline init / start / stop
// ─────────────────────────────────────────────────────────────────────────────
static void audio_pipeline_init(void)
{
    nrfx_err_t err;

    // 1. Connect SAADC IRQ (nrfx needs manual connection in Zephyr)
    IRQ_CONNECT(SAADC_IRQn, 5, nrfx_isr, nrfx_saadc_irq_handler, 0);
    irq_enable(SAADC_IRQn);

    // 2. Init SAADC
    err = nrfx_saadc_init(5);
    if (err != NRFX_SUCCESS) {
        printf("SAADC init failed: %d\n", err);
        return;
    }

    // 3. Configure channel 0 (AIN0 = microphone)
    nrfx_saadc_channel_t ch = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);
    ch.channel_config.gain      = NRF_SAADC_GAIN1_6;
    ch.channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    ch.channel_config.acq_time  = NRF_SAADC_ACQTIME_10US;
    err = nrfx_saadc_channels_config(&ch, 1);
    if (err != NRFX_SUCCESS) {
        printf("SAADC channel config failed: %d\n", err);
        return;
    }

    // 4. Advanced mode: continuous DMA with auto-restart
    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    adv.start_on_end = true;  // auto-START after each buffer completes
    err = nrfx_saadc_advanced_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT,
                                        &adv, saadc_handler);
    if (err != NRFX_SUCCESS) {
        printf("SAADC advanced mode failed: %d\n", err);
        return;
    }

    // 5. Connect TIMER2 IRQ
    IRQ_CONNECT(TIMER2_IRQn, 5, nrfx_isr, nrfx_timer_2_irq_handler, 0);

    // 6. Init Timer2 @ 16 MHz base, CC=1000 → 16 kHz
    nrfx_timer_config_t tcfg = NRFX_TIMER_DEFAULT_CONFIG(16000000);
    tcfg.mode      = NRF_TIMER_MODE_TIMER;
    tcfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
    err = nrfx_timer_init(&sample_timer, &tcfg, timer_handler);
    if (err != NRFX_SUCCESS) {
        printf("Timer init failed: %d\n", err);
        return;
    }
    nrfx_timer_extended_compare(&sample_timer, NRF_TIMER_CC_CHANNEL0,
                                 TIMER_CC_16KHZ,
                                 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                 false);  // no interrupt needed

    // 7. PPI: Timer2 COMPARE[0] → SAADC SAMPLE
    err = nrfx_gppi_channel_alloc(&ppi_channel);
    if (err != NRFX_SUCCESS) {
        printf("PPI alloc failed: %d\n", err);
        return;
    }
    nrfx_gppi_channel_endpoints_setup(
        ppi_channel,
        nrfx_timer_compare_event_address_get(&sample_timer, NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE)
    );

    printf("Audio pipeline initialized (16 kHz, ADPCM, 164-byte packets)\n");
}

static void start_audio_pipeline(void)
{
    // Reset encoder
    enc_pred = 0;
    enc_idx  = 0;
    dc_calibrated = false;

    // Queue both DMA buffers
    nrfx_saadc_buffer_set(saadc_buf0, FRAME_SAMPLES);
    nrfx_saadc_buffer_set(saadc_buf1, FRAME_SAMPLES);

    // Arm SAADC (waits for SAMPLE tasks from PPI)
    nrfx_saadc_mode_trigger();

    // Enable PPI and start timer
    nrfx_gppi_channels_enable(BIT(ppi_channel));
    nrfx_timer_enable(&sample_timer);

    pipeline_running = true;
    printf("Audio pipeline started\n");
}

static void stop_audio_pipeline(void)
{
    nrfx_timer_disable(&sample_timer);
    nrfx_gppi_channels_disable(BIT(ppi_channel));
    nrfx_saadc_abort();

    pipeline_running = false;
    printf("Audio pipeline stopped\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// IMA ADPCM encoder: 320 × int16 PCM → 164-byte packet
// ─────────────────────────────────────────────────────────────────────────────
static void adpcm_encode_frame(const int16_t *raw, uint8_t *pkt)
{
    // Block header: decoder syncs from these values
    pkt[0] = (uint8_t)(enc_pred & 0xFF);
    pkt[1] = (uint8_t)((enc_pred >> 8) & 0xFF);
    pkt[2] = (uint8_t)enc_idx;
    pkt[3] = 0;

    for (int i = 0; i < FRAME_SAMPLES; i++) {
        // DC removal + scale 12-bit centered → 16-bit range
        int32_t pcm = ((int32_t)raw[i] - dc_bias) * 16;
        if (pcm >  32767) pcm =  32767;
        if (pcm < -32768) pcm = -32768;

        // Encode one sample → 4-bit nibble
        int32_t diff = (int16_t)pcm - enc_pred;
        uint8_t nibble = 0;
        int16_t step = ima_step[enc_idx];

        if (diff < 0) { nibble = 8; diff = -diff; }
        if (diff >= step)     { nibble |= 4; diff -= step; }
        if (diff >= step / 2) { nibble |= 2; diff -= step / 2; }
        if (diff >= step / 4) { nibble |= 1; }

        // Decode to update predictor (must match decoder exactly)
        int32_t delta = step >> 3;
        if (nibble & 4) delta += step;
        if (nibble & 2) delta += step >> 1;
        if (nibble & 1) delta += step >> 2;
        if (nibble & 8) delta = -delta;

        enc_pred += (int16_t)delta;
        if (enc_pred >  32767) enc_pred =  32767;
        if (enc_pred < -32768) enc_pred = -32768;

        int16_t new_idx = enc_idx + ima_idx_adj[nibble];
        if (new_idx < 0)  new_idx = 0;
        if (new_idx > 88) new_idx = 88;
        enc_idx = (int8_t)new_idx;

        // Pack two nibbles per byte (low nibble first)
        int bpos = ADPCM_HDR_BYTES + (i / 2);
        if (i & 1) {
            pkt[bpos] |= (nibble << 4);
        } else {
            pkt[bpos] = nibble;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Button handler (power off / toggle advertising)
// ─────────────────────────────────────────────────────────────────────────────
static void button_work_handler(struct k_work *work)
{
    if (k_timer_status_get(&button_timer) > 0) {
        printf("Long press → power off\n");
        gpio_pin_set_dt(&pwr_En, 0);
        while (1) { k_sleep(K_FOREVER); }
    } else {
        printf("Short press → toggle advertising\n");
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
    k_timer_stop(&button_timer);
}

void button_timer_expiry(struct k_timer *timer_id)
{
    long_press_detected = true;
    k_work_submit(&button_work);
}

void input_pin_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (gpio_pin_get_dt(&pair_pin)) {
        k_timer_start(&button_timer, K_SECONDS(3), K_NO_WAIT);
    } else {
        k_work_submit(&button_work);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LED helpers
// ─────────────────────────────────────────────────────────────────────────────
void update_led_strip(uint8_t r, uint8_t g, uint8_t b)
{
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
    case BLE_STATE_ADVERTISING: update_led_strip(255, 255, 0);   break;
    case BLE_STATE_CONNECTING:  update_led_strip(255, 165, 0);   break;
    case BLE_STATE_CONNECTED:   update_led_strip(0, 255, 0);     break;
    default:                    update_led_strip(255, 0, 0);     break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────────────────────
int main(void)
{
    printf("Mic-Sense Audio Streaming (16 kHz ADPCM)\n");
    update_led_strip(0, 255, 255);

    // Button / power GPIO
    k_work_init(&button_work, button_work_handler);
    k_timer_init(&button_timer, button_timer_expiry, NULL);
    update_led_state(BLE_STATE_IDLE);

    if (!device_is_ready(pwr_En.port)) { printf("GPIO not ready\n"); return 0; }
    gpio_pin_configure_dt(&pwr_En, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&pwr_En, 1);

    if (!gpio_is_ready_dt(&pair_pin)) return 0;
    gpio_pin_configure_dt(&pair_pin, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&pair_pin, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&input_cb_data, input_pin_isr, BIT(pair_pin.pin));
    gpio_add_callback(pair_pin.port, &input_cb_data);

    // BLE
    if (init_ble() == 0) printf("BLE initialized\n");
    else                  printf("BLE init failed\n");

    advertising_active = false;
    update_led_strip(255, 0, 0);

    // Audio hardware pipeline (Timer2 + PPI + SAADC DMA)
    audio_init();
    audio_pipeline_init();

    printf("Ready. Waiting for BLE stream command...\n");

    // ─── Main loop: wait for DMA frames → ADPCM encode → BLE notify ────
    while (1) {
        // Not streaming: stop pipeline if running, sleep
        if (!audio_stream_active || !my_connection) {
            if (pipeline_running) {
                stop_audio_pipeline();
            }
            k_msleep(100);
            continue;
        }

        // Start pipeline on demand
        if (!pipeline_running) {
            start_audio_pipeline();
        }

        // Wait for next DMA-filled frame (20 ms cadence)
        if (k_sem_take(&frame_sem, K_MSEC(50)) != 0) {
            continue;
        }

        // First frame: calibrate DC bias, don't send
        if (!dc_calibrated) {
            int32_t sum = 0;
            for (int i = 0; i < FRAME_SAMPLES; i++) {
                sum += ready_buf[i];
            }
            dc_bias = (int16_t)(sum / FRAME_SAMPLES);
            dc_calibrated = true;
            printf("DC bias: %d raw (~%d mV)\n", dc_bias, (dc_bias * 3600) / 4096);
            continue;
        }

        // ADPCM encode 320 samples → 164-byte packet
        uint8_t pkt[ADPCM_PKT_SIZE];
        adpcm_encode_frame(ready_buf, pkt);

        // BLE notify
        bt_gatt_notify(my_connection, audio_data_attr, pkt, ADPCM_PKT_SIZE);
    }

    return 0;
}
