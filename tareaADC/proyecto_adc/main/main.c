#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "ADC_PROJECT"

// Pines ADC (GPIOs)
#define NTC_CH   ADC_CHANNEL_4   // GPIO32
#define POT_CH   ADC_CHANNEL_5   // GPIO33

// Parámetros del NTC
#define BETA 3950.0f
#define T0 298.15f       // 25 °C en Kelvin
#define R0 10000.0f      // 10k a 25 °C
#define R_SERIES 10000.0f

// Definiciones PWM (LED)
#define LED_GPIO 2
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t cali_handle = NULL;

// --- Inicialización del ADC moderno ---
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, NTC_CH, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_CH, &config));

    // Calibración
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle));

    ESP_LOGI(TAG, "ADC configurado correctamente.");
}

// --- Lectura promedio del ADC ---
static int read_adc_avg(adc_channel_t channel)
{
    int sum = 0;
    int raw;
    for (int i = 0; i < 16; i++) {
        adc_oneshot_read(adc1_handle, channel, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return sum / 16;
}

// --- Conversión a milivoltios ---
static float raw_to_voltage(int raw)
{
    int mV;
    adc_cali_raw_to_voltage(cali_handle, raw, &mV);
    return (float)mV;
}

// --- Conversión a temperatura ---
static float calculate_temperature(float v_ntc)
{
    float r_ntc = (v_ntc * R_SERIES) / (3300.0f - v_ntc);
    float invT = (1.0f / T0) + (1.0f / BETA) * logf(r_ntc / R0);
    return (1.0f / invT) - 273.15f;
}

// --- Inicialización PWM ---
static void pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .gpio_num = LED_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
}

void app_main(void)
{
    adc_init();
    pwm_init();

    while (1)
    {
        int pot_raw = read_adc_avg(POT_CH);
        int ntc_raw = read_adc_avg(NTC_CH);

        float pot_mV = raw_to_voltage(pot_raw);
        float ntc_mV = raw_to_voltage(ntc_raw);

        float temp = calculate_temperature(ntc_mV);

        printf("POT: %.1f mV | NTC: %.1f mV | Temp: %.2f °C\n",
               pot_mV, ntc_mV, temp);

        uint32_t duty = (pot_raw * 255) / 4095;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
