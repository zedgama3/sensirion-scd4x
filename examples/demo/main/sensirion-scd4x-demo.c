#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

#include "driver/i2c_master.h"


#define TAG "Cultivator"

void app_main(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = 7,
        .sda_io_num = 6,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    sensirion_i2c_hal_init(i2c_mst_config);

    ESP_LOGD(TAG, "Clean up potential SCD40 states");
    scd4x_wake_up();
    ESP_LOGD(TAG, "scd4x_stop_periodic_measurement()");
    scd4x_stop_periodic_measurement();
    ESP_LOGD(TAG, "scd4x_reinit()");
    scd4x_reinit();

    // Optional: Disable automatic self-calibration
    uint16_t automatic_self_calibration_enabled;
    scd4x_get_automatic_self_calibration(&automatic_self_calibration_enabled);
    if (automatic_self_calibration_enabled) {
        ESP_LOGD(TAG, "Disabling automatic self-calibration");
        ESP_ERROR_CHECK(scd4x_set_automatic_self_calibration(false));
        ESP_ERROR_CHECK(scd4x_persist_settings());
    }

    uint16_t serial_0;
    uint16_t serial_1;
    uint16_t serial_2;
    esp_err_t error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
    if (error) {
        printf("Error executing scd4x_get_serial_number(): %i\n", error);
    } else {
        printf("serial: 0x%04x%04x%04x\n", serial_0, serial_1, serial_2);
    }

    // Start Measurement

    error = scd4x_start_periodic_measurement();
    if (error) {
        printf("Error executing scd4x_start_periodic_measurement(): %i\n",
               error);
    }

    printf("Waiting for first measurement... (5 sec)\n");

    // Print the header
    printf("+---------------------+---------+-------------+----------+\n");
    printf("|        Date         |   CO2   | Temperature | Humidity |\n");
    printf("+---------------------+---------+-------------+----------+\n");

    for (;;) {
        // Read Measurement
        vTaskDelay(100 / portTICK_PERIOD_MS);
        bool data_ready_flag = false;
        error = scd4x_get_data_ready_flag(&data_ready_flag);
        if (error) {
            ESP_LOGE(TAG, "Error executing scd4x_get_data_ready_flag(): %i\n", error);
            continue;
        }
        if (!data_ready_flag) {
            continue;
        }

        uint16_t co2;
        int32_t temperature;
        int32_t humidity;
        error = scd4x_read_measurement(&co2, &temperature, &humidity);
        if (error) {
            printf("Error executing scd4x_read_measurement(): %i\n", error);
        } else if (co2 == 0) {
            printf("Invalid sample detected, skipping.\n");
        } else {
            float temperature_C = temperature / 1000.0;
            float humidity_RH = humidity / 1000.0;

            // printf("CO2: %u\n", co2);
            // printf("Temperature: %.1f °C\n", temperature_C);
            // printf("Humidity: %.1f RH\n", humidity_RH);

            // Get the current time
            time_t now = time(NULL);
            struct tm *t = localtime(&now);

             // Format the current time
            char time_str[20];
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", t);
            printf("| %s | %7u | %9.1f°C | %7.1f%% |\n", time_str, co2, temperature_C, humidity_RH);

        }
    }
}
