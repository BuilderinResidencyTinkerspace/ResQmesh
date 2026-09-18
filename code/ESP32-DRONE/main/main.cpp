/**
 * @file main.cpp
 * @brief Application Entry Point for ESP32-S3 Drone Flight Controller.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "config.h"
#include "flight_controller.h"
#include "cli.h"

static const char *TAG = "MAIN";

// Global Flight Controller & CLI instances
static FlightController g_flight_controller;
static SerialCLI g_cli(g_flight_controller);

/**
 * @brief Background task on Core 0 to drive the visual status LED.
 * Blink codes:
 * - DISARMED: Slow pulse (1 Hz)
 * - ARMED / FLIGHT: Solid ON
 * - FAILSAFE / ERROR: Rapid flash (10 Hz)
 * - LOW BATTERY: Double flash
 */
static void statusLedTask(void *arg) {
    FlightController *fc = reinterpret_cast<FlightController*>(arg);

    // Initialize LED GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_STATUS_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    auto set_led = [](bool on) {
#if LED_ACTIVE_LOW
        gpio_set_level((gpio_num_t)PIN_STATUS_LED, on ? 0 : 1);
#else
        gpio_set_level((gpio_num_t)PIN_STATUS_LED, on ? 1 : 0);
#endif
    };

    while (true) {
        DroneState state = fc->getSafety().getState();
        bool batt_low = fc->getBattery().isLow();

        if (state == STATE_ARMED || state == STATE_FLIGHT) {
            // Armed: Solid ON
            set_led(true);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (state == STATE_FAILSAFE || state == STATE_ERROR || state == STATE_EMERGENCY_STOP) {
            // Failsafe/Error: Rapid alarm blink (10 Hz)
            set_led(true);
            vTaskDelay(pdMS_TO_TICKS(50));
            set_led(false);
            vTaskDelay(pdMS_TO_TICKS(50));
        } else if (batt_low) {
            // Low Battery: Double pulse
            set_led(true);
            vTaskDelay(pdMS_TO_TICKS(80));
            set_led(false);
            vTaskDelay(pdMS_TO_TICKS(80));
            set_led(true);
            vTaskDelay(pdMS_TO_TICKS(80));
            set_led(false);
            vTaskDelay(pdMS_TO_TICKS(600));
        } else {
            // Normal Disarmed: Slow heartbeat (1 Hz)
            set_led(true);
            vTaskDelay(pdMS_TO_TICKS(100));
            set_led(false);
            vTaskDelay(pdMS_TO_TICKS(900));
        }
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "  %s v%s - Booting Flight Controller", FIRMWARE_NAME, FIRMWARE_VERSION);
    ESP_LOGI(TAG, "  Platform: %s", HARDWARE_TARGET);
    ESP_LOGI(TAG, "=======================================================");

    // 1. Initialize NVS (required for Wi-Fi and persistent calibration)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Flight Controller
    if (!g_flight_controller.init()) {
        ESP_LOGE(TAG, "CRITICAL: Flight controller initialization failed! Halting.");
        g_flight_controller.emergencyStop();
        return;
    }

    // 3. Start 500 Hz Flight Control loop on Core 1
    if (!g_flight_controller.start()) {
        ESP_LOGE(TAG, "CRITICAL: Failed to launch flight loop FreeRTOS task!");
        g_flight_controller.emergencyStop();
        return;
    }

    // 4. Start Serial CLI on Core 0
    g_cli.start();

    // 5. Start Status LED task on Core 0
    xTaskCreatePinnedToCore(
        statusLedTask,
        "led_task",
        2048,
        &g_flight_controller,
        1,
        nullptr,
        0
    );

    ESP_LOGI(TAG, "Flight controller running. Core 1: Flight Loop (500Hz) | Core 0: Comms & CLI.");
}
