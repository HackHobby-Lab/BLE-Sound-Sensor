
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "micsense_service.h"

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


int main(void)
{
    int err;

    if (init_ble() == 0) {
        printf("BLE Initialized successfully.\n");
    } else {
        printf("BLE Initialization failed.\n");
    }
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
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
}
