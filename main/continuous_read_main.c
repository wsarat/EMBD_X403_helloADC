/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include <math.h>

// ADC pin GPIO 3 ADC1_CH2

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN                    256

static adc_channel_t channel[1] = {ADC_CHANNEL_2};

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";


#define SR_DS   9
#define SR_OE_  10
#define SR_STCP 11 // output clock
#define SR_SHCP 12 // shift register clock
//#define SR_MR_  N/A

#define SR_DELAY    1

#define LEDBAR_9    13
#define LEDBAR_10   14

static void ledBar(uint16_t val);
static void ledBar_setup(void)
{
    gpio_reset_pin(SR_DS);
    gpio_reset_pin(SR_OE_);
    gpio_reset_pin(SR_STCP);
    gpio_reset_pin(SR_SHCP);
    gpio_reset_pin(LEDBAR_9);
    gpio_reset_pin(LEDBAR_10);

    gpio_set_direction(SR_DS, GPIO_MODE_OUTPUT);
    gpio_set_direction(SR_OE_, GPIO_MODE_OUTPUT);
    gpio_set_direction(SR_STCP, GPIO_MODE_OUTPUT);
    gpio_set_direction(SR_SHCP, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDBAR_9, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDBAR_10, GPIO_MODE_OUTPUT);

    gpio_set_level(SR_OE_, 0); // output enable
    gpio_set_level(SR_SHCP, 0);
    gpio_set_level(SR_STCP, 0);   
    gpio_set_level(SR_DS, 0); 

    gpio_set_level(LEDBAR_9, 1); 
    gpio_set_level(LEDBAR_10, 1); 

    ledBar(0);
}

uint16_t lastValue = 1024;
static void ledBar(uint16_t val) 
{
    if (val == lastValue)
        return;
    lastValue = val;

    gpio_set_level(SR_SHCP, 0);
    gpio_set_level(SR_STCP, 0);

    printf("value = %d => ", val);
    bool bit;
    for (int i=0; i<8; i++) {
        bit = (val & 0x1 << i);
        if (i % 4 == 0) printf(" "); // space
        printf("%s", bit? "1":"0");
        
        gpio_set_level(SR_DS, !bit); // invert due to wiring
        //vTaskDelay(pdMS_TO_TICKS(SR_DELAY));

        gpio_set_level(SR_SHCP, 1);
        vTaskDelay(pdMS_TO_TICKS(SR_DELAY));   

        gpio_set_level(SR_SHCP, 0);
        vTaskDelay(pdMS_TO_TICKS(SR_DELAY));   
    }
    printf(" "); // space

    bit = (val & 0x1 << 9);
    printf("%s", bit? "1":"0");
    gpio_set_level(LEDBAR_9, !bit);

    bit = (val & 0x1 << 10);
    printf("%s", bit? "1":"0");
    gpio_set_level(LEDBAR_10, !bit);
    printf("\n");

    gpio_set_level(SR_STCP, 1);
    vTaskDelay(pdMS_TO_TICKS(SR_DELAY));  

    gpio_set_level(SR_STCP, 0);
    vTaskDelay(pdMS_TO_TICKS(SR_DELAY));  
}

static void ledBarLevel(uint32_t val, uint32_t max) 
{
    double level = (10 * val)/max;
    uint16_t barVal = 0;
    for (int i=0; i<=level; i++) {
        barVal |= (0x1 << i);
    }
    ledBar(barVal);

    return;
}


static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void app_main(void)
{
    ledBar_setup();
    
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        while (1) {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (void*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                     if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        //ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                        //ledBar(data);
                        ledBarLevel(data, 4095);
                    } else {
                        ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
