/**
 * @file motors.cpp
 * @brief LEDC PWM Motor Controller Implementation with Hardware Boot-Safety.
 */

#include "motors.h"
#include <cmath>
#include <algorithm>

#if defined(ESP_PLATFORM)
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
static const char *TAG = "MOTORS";

static const ledc_channel_t LEDC_CHANNELS[4] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3
};

static const int MOTOR_PINS[4] = {
    PIN_MOTOR_FL,
    PIN_MOTOR_FR,
    PIN_MOTOR_RR,
    PIN_MOTOR_RL
};
#else
#include <cstdio>
#define ESP_LOGI(tag, fmt, ...) printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR][%s] " fmt "\n", tag, ##__VA_ARGS__)
static const char *TAG = "MOTORS_HOST";
#endif

MotorController::MotorController()
    : is_armed_(false) {
    for (int i = 0; i < 4; i++) {
        current_duty_[i] = 0;
    }
}

MotorController::~MotorController() {
    stopAll();
}

bool MotorController::init() {
    ESP_LOGI(TAG, "Initializing motor PWM outputs at %d Hz (10-bit resolution)...", MOTOR_PWM_FREQ_HZ);

#if defined(ESP_PLATFORM)
    // 1. Critical Boot Safety: Force all motor GPIOs to LOW with pull-downs BEFORE starting PWM
    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << MOTOR_PINS[i]);
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level((gpio_num_t)MOTOR_PINS[i], 0);
    }

    // 2. Configure LEDC Timer
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = (ledc_timer_bit_t)MOTOR_PWM_BITS;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = MOTOR_PWM_FREQ_HZ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: 0x%x", err);
        return false;
    }

    // 3. Configure all 4 LEDC Channels with initial duty = 0
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_conf = {};
        ch_conf.gpio_num = MOTOR_PINS[i];
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel = LEDC_CHANNELS[i];
        ch_conf.intr_type = LEDC_INTR_DISABLE;
        ch_conf.timer_sel = LEDC_TIMER_0;
        ch_conf.duty = 0;
        ch_conf.hpoint = 0;

        err = ledc_channel_config(&ch_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed for channel %d: 0x%x", i, err);
            return false;
        }
    }
#endif

    stopAll();
    is_armed_ = false;
    ESP_LOGI(TAG, "Motor controller initialized. ALL MOTORS GUARANTEED OFF.");
    return true;
}

void MotorController::arm() {
    is_armed_ = true;
#if MOTOR_PWM_IDLE_ARM > 0
    // Spin motors gently at idle to indicate armed state to operator
    setAll((float)MOTOR_PWM_IDLE_ARM);
#else
    stopAll();
#endif
    ESP_LOGI(TAG, "Motors ARMED.");
}

void MotorController::disarm() {
    is_armed_ = false;
    stopAll();
    ESP_LOGI(TAG, "Motors DISARMED.");
}

void MotorController::emergencyStop() {
    is_armed_ = false;
    stopAll();
    ESP_LOGW(TAG, "EMERGENCY STOP EXECUTED! Motors zeroed.");
}

void MotorController::setMotor(uint8_t index, float duty) {
    if (index >= 4) return;

    if (!is_armed_) {
        writeDutyHardware(index, 0);
        current_duty_[index] = 0;
        return;
    }

    if (std::isnan(duty) || std::isinf(duty) || duty < 0.0f) {
        duty = 0.0f;
    }

    // Enforce limits
    if (duty > (float)MOTOR_PWM_MAX_FLIGHT) {
        duty = (float)MOTOR_PWM_MAX_FLIGHT;
    }

    uint32_t duty_int = (uint32_t)duty;
    current_duty_[index] = duty_int;
    writeDutyHardware(index, duty_int);
}

void MotorController::setAll(float duty) {
    for (uint8_t i = 0; i < 4; i++) {
        setMotor(i, duty);
    }
}

void MotorController::applyOutputs(const float outputs[4]) {
    for (uint8_t i = 0; i < 4; i++) {
        setMotor(i, outputs[i]);
    }
}

void MotorController::stopAll() {
    for (uint8_t i = 0; i < 4; i++) {
        current_duty_[i] = 0;
        writeDutyHardware(i, 0);
    }
}

bool MotorController::testMotor(uint8_t motor_index, float duty_percent) {
#if !MOTOR_TEST_ENABLED
    ESP_LOGW(TAG, "Motor test rejected: MOTOR_TEST_ENABLED is 0 in config.h");
    return false;
#else
    if (is_armed_) {
        ESP_LOGE(TAG, "Motor test rejected: drone is armed! Disarm first.");
        return false;
    }
    if (motor_index >= 4) {
        ESP_LOGE(TAG, "Motor test rejected: invalid index %d", motor_index);
        return false;
    }
    if (duty_percent < 0.0f || duty_percent > 25.0f) {
        ESP_LOGE(TAG, "Motor test rejected: duty %.1f%% exceeds 25.0%% safety ceiling", duty_percent);
        return false;
    }

    uint32_t duty_val = (uint32_t)((duty_percent / 100.0f) * (float)MOTOR_PWM_MAX_VALUE);
    ESP_LOGI(TAG, "Bench testing Motor %d at duty %u (%.1f%%)", motor_index + 1, duty_val, duty_percent);
    writeDutyHardware(motor_index, duty_val);
    return true;
#endif
}

void MotorController::writeDutyHardware(uint8_t channel, uint32_t duty) {
    if (channel >= 4) return;
    if (duty > MOTOR_PWM_MAX_VALUE) duty = MOTOR_PWM_MAX_VALUE;

#if defined(ESP_PLATFORM)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNELS[channel], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNELS[channel]);
#else
    current_duty_[channel] = duty;
#endif
}
