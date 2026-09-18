/**
 * @file main.cpp
 * @brief ESP-NOW Quadcopter Remote Transmitter Firmware.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "config.h"
#include "packet.h"

static const char *TAG = "TX_CONTROLLER";

static uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// State variables for joystick / keyboard control
static ControlPacket s_tx_packet;
static uint32_t s_sequence = 0;

static void initGPIO() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_SWITCH_ARM) |
                           (1ULL << PIN_SWITCH_MODE) |
                           (1ULL << PIN_BUTTON_EMERGENCY);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

static bool initESPNOW() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed!");
        return false;
    }

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, s_broadcast_mac, 6);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer!");
        return false;
    }

    ESP_LOGI(TAG, "ESP-NOW transmitter initialized on channel %d", ESPNOW_WIFI_CHANNEL);
    return true;
}

static void txTask(void *arg) {
    ESP_LOGI(TAG, "Starting 50 Hz Transmitter loop...");
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // Read hardware toggle switches (active LOW)
        int arm_sw = gpio_get_level((gpio_num_t)PIN_SWITCH_ARM);
        int mode_sw = gpio_get_level((gpio_num_t)PIN_SWITCH_MODE);
        int emg_btn = gpio_get_level((gpio_num_t)PIN_BUTTON_EMERGENCY);

        if (emg_btn == 0) {
            s_tx_packet.arm = 2; // Emergency stop
            s_tx_packet.throttle = 0;
        } else if (arm_sw == 0) {
            s_tx_packet.arm = 1; // Armed
        } else {
            s_tx_packet.arm = 0; // Disarmed
        }

        s_tx_packet.mode = (mode_sw == 0) ? 1 : 0; // 0=Angle, 1=Rate
        s_tx_packet.sequence = ++s_sequence;

        // Compute CRC over the packet body
        size_t payload_len = sizeof(ControlPacket) - sizeof(uint16_t);
        s_tx_packet.crc = computePacketCRC((const uint8_t*)&s_tx_packet, payload_len);

        // Transmit packet over ESP-NOW
        esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t*)&s_tx_packet, sizeof(ControlPacket));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Packet send error: 0x%x", err);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TRANSMIT_INTERVAL_MS));
    }
}

/**
 * @brief Interactive Serial task allowing joystick control via terminal keys
 */
static void serialControlTask(void *arg) {
    printf("\r\n=======================================================\r\n");
    printf("  ESP32 Drone Transmitter Console\r\n");
    printf("  Keyboard Controls (or wire analog joysticks):\r\n");
    printf("    [Space] : EMERGENCY STOP\r\n");
    printf("    [a]     : Arm / Disarm toggle\r\n");
    printf("    [w/s]   : Throttle +/- 5%%\r\n");
    printf("    [i/k]   : Pitch Forward / Back\r\n");
    printf("    [j/l]   : Roll Left / Right\r\n");
    printf("    [u/o]   : Yaw Left / Right\r\n");
    printf("    [x]     : Center all sticks (zero roll/pitch/yaw)\r\n");
    printf("    [c]     : Send Sensor Calibration trigger\r\n");
    printf("=======================================================\r\n\r\n");

    while (true) {
        int c = getchar();
        if (c != EOF && c > 0) {
            switch (c) {
                case ' ': // Emergency stop
                    s_tx_packet.arm = 2;
                    s_tx_packet.throttle = 0;
                    printf(">>> [EMERGENCY STOP SENT] <<<\r\n");
                    break;
                case 'a':
                    s_tx_packet.arm = (s_tx_packet.arm == 1) ? 0 : 1;
                    printf("Arm Switch: %s\r\n", s_tx_packet.arm == 1 ? "ARMED" : "DISARMED");
                    break;
                case 'w':
                    if (s_tx_packet.throttle <= 950) s_tx_packet.throttle += 50;
                    printf("Throttle: %u / 1000\r\n", s_tx_packet.throttle);
                    break;
                case 's':
                    if (s_tx_packet.throttle >= 50) s_tx_packet.throttle -= 50;
                    else s_tx_packet.throttle = 0;
                    printf("Throttle: %u / 1000\r\n", s_tx_packet.throttle);
                    break;
                case 'i':
                    s_tx_packet.pitch = -250; // Pitch forward (nose down)
                    printf("Pitch Forward (-250)\r\n");
                    break;
                case 'k':
                    s_tx_packet.pitch = 250;  // Pitch back (nose up)
                    printf("Pitch Back (+250)\r\n");
                    break;
                case 'j':
                    s_tx_packet.roll = -250;  // Roll left
                    printf("Roll Left (-250)\r\n");
                    break;
                case 'l':
                    s_tx_packet.roll = 250;   // Roll right
                    printf("Roll Right (+250)\r\n");
                    break;
                case 'u':
                    s_tx_packet.yaw = -250;   // Yaw CCW
                    printf("Yaw Left (-250)\r\n");
                    break;
                case 'o':
                    s_tx_packet.yaw = 250;    // Yaw CW
                    printf("Yaw Right (+250)\r\n");
                    break;
                case 'x':
                    s_tx_packet.roll = 0;
                    s_tx_packet.pitch = 0;
                    s_tx_packet.yaw = 0;
                    printf("Sticks Centered (R=0, P=0, Y=0)\r\n");
                    break;
                case 'c':
                    s_tx_packet.mode = 2; // Trigger calibration
                    printf("Sent Calibration Command\r\n");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    s_tx_packet.mode = 0;
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Initializing Drone Remote Transmitter...");

    nvs_flash_init();
    initGPIO();
    initESPNOW();

    memset(&s_tx_packet, 0, sizeof(s_tx_packet));

    xTaskCreate(txTask, "tx_task", 4096, nullptr, 5, nullptr);
    xTaskCreate(serialControlTask, "serial_task", 4096, nullptr, 3, nullptr);
}
