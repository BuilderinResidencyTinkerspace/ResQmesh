/**
 * @file firmware.ino
 * @brief Arduino Sketch Entrypoint for ESP32-DRONE Flight Controller.
 */

#include <Arduino.h>
#include "nvs_flash.h"
#include "config.h"
#include "flight_controller.h"
#include "cli.h"

static FlightController g_flight_controller;
static SerialCLI g_cli(g_flight_controller);

void setup() {
    Serial.begin(115200);

    // Give serial monitor a moment to attach
    delay(1500);

    Serial.println("\r\n=======================================================");
    Serial.println("  ESP32-S3 Drone Flight Controller - Booting");
    Serial.println("=======================================================");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Initialize Flight Controller
    if (!g_flight_controller.init()) {
        Serial.println("[ERROR] Flight controller initialization failed! Halting.");
        Serial.println("  -> Reason: Physical MPU-9250 sensor not detected on I2C (SDA=D4/GPIO5, SCL=D5/GPIO6).");
        Serial.println("  -> Tip: To test Swarm networking & CLI on bare desk boards without an IMU,");
        Serial.println("          set #define IMU_BENCH_TEST_MODE 1 in config.h and reflash!");
        g_flight_controller.emergencyStop();
        return;
    }

    // Launch 500 Hz deterministic flight loop on Core 1
    if (!g_flight_controller.start()) {
        Serial.println("[ERROR] Failed to start flight loop task!");
        g_flight_controller.emergencyStop();
        return;
    }

    // Launch Serial CLI
    g_cli.start();

    Serial.println("[INFO] Flight controller ready. Core 1: 500Hz Loop | Core 0: Comms & CLI.");
    Serial.print("\r\n> ");
}

void loop() {
    // Poll USB Serial for commands
    g_cli.update();
    delay(5);
}
