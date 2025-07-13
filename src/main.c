
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

#define STRIP_NODE        DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS  DT_PROP(STRIP_NODE, chain_length)
#define DELAY_TIME        K_MSEC(5)

struct led_rgb pixels[STRIP_NUM_PIXELS];
const struct device *strip = DEVICE_DT_GET(STRIP_NODE);


#define EN_PIN_NODE  DT_NODELABEL(user_output_pin)

static const struct gpio_dt_spec pwr_En = GPIO_DT_SPEC_GET(EN_PIN_NODE, gpios);

#define PAIR_PIN DT_NODELABEL(user_input_pin)
static const struct gpio_dt_spec pair_pin = GPIO_DT_SPEC_GET(PAIR_PIN, gpios);

static struct gpio_callback input_cb_data;

bool status = false;
int count = 0;
#define AUDIO_BUFFER_SIZE 16000 // e.g., 1 second at 16 kHz
int16_t audio_buffer[AUDIO_BUFFER_SIZE];
volatile uint32_t audio_write_index = 0;



#define SLEEP_TIME_MS 100
float db =0.0;
int16_t db_int = 0;
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 12
#define ADC_CHANNEL 0
#define ADC_PORT SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN0
#define ADC_REFERENCE ADC_REF_INTERNAL             // 0.6V
#define ADC_GAIN ADC_GAIN_1_6                      // ADC_REFERENCE * 5


#define BATTERY_ADC_CHANNEL 1 //Battery 
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
    .resolution = ADC_RESOLUTION
};

//ADC Battery Sequence
int16_t battery_sample[1];

struct adc_sequence battery_sequence = {
    .channels = BIT(BATTERY_ADC_CHANNEL),
    .buffer = battery_sample,
    .buffer_size = sizeof(battery_sample),
    .resolution = ADC_RESOLUTION
};

static const struct bt_data ad[] = 
{
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SET_THRESHOLD_SERVICE_VAL),
};

void input_pin_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printf("Interrupt! Pin state: %d\n", gpio_pin_get_dt(&pair_pin));
    status != status;
    count = count +1;
    
}

void update_led_strip(uint8_t r, uint8_t g, uint8_t b)
{
    // Set the RGB values for all the pixels
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i].r = r;
        pixels[i].g = g;
        pixels[i].b = b;
    }

    // Update the LED strip
    led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
}


int main(void)
{
    int err;
    printk("Startup\n");
    update_led_strip(0, 0, 255);

     if (!device_is_ready(pwr_En.port)) {
        printk("GPIO port not ready\n");
        return;
    }
    gpio_pin_configure_dt(&pwr_En, GPIO_OUTPUT_ACTIVE); // Start HIGH (ACTIVE)
    gpio_pin_set_dt(&pwr_En, 1); // Set HIGH again


    // Check if the device is ready
    if (!gpio_is_ready_dt(&pair_pin)) {
        return;
    }

    // Configure the pin as input
    gpio_pin_configure_dt(&pair_pin, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&pair_pin, GPIO_INT_EDGE_TO_ACTIVE);
    // Initialize and add the callback
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
        err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err)
        {
            printf("Advertising failed to start(err %d)\n", err);
            return err;
        }

    if (!device_is_ready(adc_dev))
    {
        printf("ADC Device not read\n");
        return;
    }

    err = adc_channel_setup(adc_dev, &chl0_cfg);
    if (err != 0)
    {
        printf("ADC Setup failedwith error %d.\n", err);
        return;
    }

    //ADC Battery Setup
    err = adc_channel_setup(adc_dev, &battery_ch_cfg);
    if (err != 0) {
        printf("Battery ADC Setup failed with error %d.\n", err);
        return;
    }


    while (1)
    {   
        if (count == 1) 
        {
            update_led_strip(255, 0, 0);
        }
        if (count == 2) 
        {
            update_led_strip(0, 255, 0);
        }
        if (count == 3) 
        {
            update_led_strip(0, 0, 255);
        }
        if (count == 4) 
        {
            update_led_strip(255, 255, 255);
        }
        if (count == 5) 
        {
            count = 0;
        }
        

        err = adc_read(adc_dev, &sequence);
        if (err != 0)
        {
            printf("ADC reading failed with error %d.\n", err);
            return 0;
        }

        

        int32_t mv_value = sampleBuffer[0];
        // Convert raw ADC to millivolts
        int32_t adc_vref = adc_ref_internal(adc_dev);
        adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &mv_value);

        // Store the raw ADC sample
        audio_buffer[audio_write_index++] = sampleBuffer[0];

        // Wrap around if full
        if (audio_write_index >= AUDIO_BUFFER_SIZE) {
            audio_write_index = 0; // Circular buffer
        }
        printf("%d\n", audio_buffer[audio_write_index]);


        // Remove fixed DC bias (~1650mV if Vcc = 3.3V)
        int32_t ac_component = mv_value - 1500; // Centered around 0

        // Take absolute value (positive only)
        if (ac_component < 0)
            ac_component = -ac_component;

        // Now do dB calculation
        if (ac_component == 0)
            ac_component = 1; // avoid log10(0)
        
        db = 20.0f * log10f((float)ac_component / 1.0f);
        if (db >= threshold_value)      
        {   
            alertThreshold = 1;
            if (my_connection) {
                int err = bt_gatt_notify(my_connection, alert_threshold_attr, &alertThreshold, sizeof(alertThreshold));
                if (err) {
                    printk("Failed to notify (err %d)\n", err);
                } else {
                    printk("Notification sent: %d\n", alertThreshold);
                }
            }
        }
        else{
            alertThreshold = 0;
        }

        
        if (sound_streaming_enabled > 0)      
        { 
            if (my_connection) {
                db_int = (int16_t)(db);
                int err = bt_gatt_notify(my_connection, getStreamService_attr, &db_int, sizeof(db_int));
                if (err) {
                    printk("Failed to notify (err %d)\n", err);
                } else {
                    printk("Notification sent: %d\n", db_int);
                }
            }
        }
        // else{
        //     sound_streaming_enabled = 0;
        // }

        // printf("Sound Level: %.2f dB\n", db);



         // --- Battery Voltage Read ---
         err = adc_read(adc_dev, &battery_sequence);
         if (err != 0) {
             printf("Battery ADC reading failed: %d\n", err);
             continue;
         }
 
         int32_t battery_mv = battery_sample[0];
         int32_t battery_vref = adc_ref_internal(adc_dev);
         adc_raw_to_millivolts(battery_vref, ADC_GAIN, ADC_RESOLUTION, &battery_mv);
 
         // If using a voltage divider (e.g., R1 = R2), multiply accordingly:
         float battery_voltage = battery_mv * 1.0f / 1000.0f; // Convert to volts
         printf("Battery Voltage: %.2f V\n", battery_voltage);

        k_msleep(SLEEP_TIME_MS);
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
