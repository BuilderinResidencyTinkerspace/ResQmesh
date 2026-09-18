/**
 * @file battery.cpp
 * @brief 1S LiPo Battery Monitor Implementation with Moving Average Filter.
 */

#include "battery.h"
#include <cmath>
#include <algorithm>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_adc/adc_oneshot.h"
#else
#include "driver/adc.h"
#endif

static const char *TAG = "BATTERY";
#else
#include <cstdio>
#define ESP_LOGI(tag, fmt, ...) printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR][%s] " fmt "\n", tag, ##__VA_ARGS__)
static const char *TAG = "BATTERY_HOST";
#endif

BatteryMonitor::BatteryMonitor(float divider_ratio)
    : divider_ratio_(divider_ratio),
      filtered_voltage_(4.20f),
      sample_idx_(0),
      buffer_filled_(false),
      use_simulation_(false),
      simulated_voltage_(4.0f) {
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        sample_buffer_[i] = 4.20f;
    }
#if defined(ESP_PLATFORM)
    adc_handle_ = nullptr;
#endif
}

BatteryMonitor::~BatteryMonitor() {
#if defined(ESP_PLATFORM)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (adc_handle_) {
        adc_oneshot_del_unit((adc_oneshot_unit_handle_t)adc_handle_);
        adc_handle_ = nullptr;
    }
#endif
#endif
}

bool BatteryMonitor::init() {
#if !BATTERY_MONITOR_ENABLED
    ESP_LOGI(TAG, "Battery monitoring bypassed (#define BATTERY_MONITOR_ENABLED 0 in config.h). Defaulting to 3.85V healthy.");
    filtered_voltage_ = 3.85f;
    return true;
#else
    ESP_LOGI(TAG, "Initializing Battery ADC (Ratio: %.2f)...", divider_ratio_);

#if defined(ESP_PLATFORM)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_unit_handle_t handle = nullptr;
    esp_err_t err = adc_oneshot_new_unit(&init_config, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: 0x%x", err);
        return false;
    }
    adc_handle_ = (void*)handle;

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12, // Measure up to ~3.1V
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(handle, (adc_channel_t)BATTERY_ADC_CHANNEL, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: 0x%x", err);
        return false;
    }
#else
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)BATTERY_ADC_CHANNEL, ADC_ATTEN_DB_11);
#endif
    ESP_LOGI(TAG, "ADC configured for GPIO%d (Channel %d)", PIN_BATTERY_ADC, BATTERY_ADC_CHANNEL);
#endif

    // Prime sample buffer
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        readVoltage();
    }
    return true;
#endif
}

float BatteryMonitor::readVoltage() {
    if (use_simulation_) {
        filtered_voltage_ = simulated_voltage_;
        return filtered_voltage_;
    }

#if !BATTERY_MONITOR_ENABLED
    filtered_voltage_ = 3.85f;
    return filtered_voltage_;
#else
    float raw_voltage = 4.0f; // Default fallback

#if defined(ESP_PLATFORM)
    int adc_raw = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (adc_handle_) {
        adc_oneshot_read((adc_oneshot_unit_handle_t)adc_handle_, 
                         (adc_channel_t)BATTERY_ADC_CHANNEL, &adc_raw);
    }
#else
    adc_raw = adc1_get_raw((adc1_channel_t)BATTERY_ADC_CHANNEL);
#endif

    // ESP32-S3 12-bit ADC: 0 to 4095 corresponds to ~0 to 3.1V with 11/12dB attenuation
    // Calibration reference voltage: 3.10V = 3100 mV
    float pin_voltage = ((float)adc_raw / 4095.0f) * 3.10f;
    raw_voltage = pin_voltage * divider_ratio_;
#endif

    // Moving average filter
    sample_buffer_[sample_idx_] = raw_voltage;
    sample_idx_ = (sample_idx_ + 1) % BATTERY_ADC_SAMPLES;
    if (sample_idx_ == 0) buffer_filled_ = true;

    float sum = 0.0f;
    int count = buffer_filled_ ? BATTERY_ADC_SAMPLES : sample_idx_;
    if (count == 0) count = 1;
    for (int i = 0; i < count; i++) {
        sum += sample_buffer_[i];
    }
    filtered_voltage_ = sum / (float)count;
#endif

    return filtered_voltage_;
}

float BatteryMonitor::getPercentage() const {
    float v = filtered_voltage_;

    // Realistic 1S LiPo piecewise discharge curve
    if (v >= 4.20f) return 100.0f;
    if (v <= 3.30f) return 0.0f;

    // Piecewise linear interpolation points: {Voltage, Percentage}
    static const struct { float volt; float pct; } lipo_curve[] = {
        {4.20f, 100.0f},
        {4.10f, 90.0f},
        {4.00f, 80.0f},
        {3.90f, 70.0f},
        {3.82f, 60.0f},
        {3.76f, 50.0f},
        {3.72f, 40.0f},
        {3.68f, 30.0f},
        {3.62f, 20.0f},
        {3.50f, 10.0f},
        {3.30f, 0.0f}
    };

    const int num_points = sizeof(lipo_curve) / sizeof(lipo_curve[0]);
    for (int i = 0; i < num_points - 1; i++) {
        if (v <= lipo_curve[i].volt && v >= lipo_curve[i+1].volt) {
            float v_range = lipo_curve[i].volt - lipo_curve[i+1].volt;
            float pct_range = lipo_curve[i].pct - lipo_curve[i+1].pct;
            float factor = (v - lipo_curve[i+1].volt) / v_range;
            return lipo_curve[i+1].pct + (factor * pct_range);
        }
    }

    return 0.0f;
}
