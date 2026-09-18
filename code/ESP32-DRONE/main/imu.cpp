/**
 * @file imu.cpp
 * @brief MPU9250 / MPU6500 9-Axis IMU Driver Implementation.
 */

#include "imu.h"
#include <cmath>
#include <cstring>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char *TAG = "IMU";
#else
#include <cstdio>
#define ESP_LOGI(tag, fmt, ...) printf("[INFO][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR][%s] " fmt "\n", tag, ##__VA_ARGS__)
static const char *TAG = "IMU_HOST";
#endif

// MPU9250 Internal Register Map
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_INT_PIN_CFG     0x37
#define REG_INT_ENABLE      0x38
#define REG_ACCEL_XOUT_H    0x3B
#define REG_TEMP_OUT_H      0x41
#define REG_GYRO_XOUT_H     0x43
#define REG_USER_CTRL       0x6A
#define REG_PWR_MGMT_1      0x6B
#define REG_PWR_MGMT_2      0x6C
#define REG_WHO_AM_I        0x75

IMU::IMU()
    : is_healthy_(false),
      error_count_(0),
      simulation_mode_(false) {
    memset(&calibration_, 0, sizeof(calibration_));
    memset(&sim_data_, 0, sizeof(sim_data_));
}

IMU::~IMU() {
#if defined(ESP_PLATFORM)
    i2c_driver_delete((i2c_port_t)I2C_PORT_NUM);
#endif
}

bool IMU::init() {
    if (simulation_mode_) {
        is_healthy_ = true;
        calibration_.is_calibrated = true;
        ESP_LOGI(TAG, "IMU initialized in SIMULATION mode");
        return true;
    }

#if defined(ESP_PLATFORM)
    // 1. Configure I2C master bus
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)PIN_I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)PIN_I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config((i2c_port_t)I2C_PORT_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: 0x%x", err);
        return false;
    }

    err = i2c_driver_install((i2c_port_t)I2C_PORT_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: 0x%x", err);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Reset device & wake up
    if (!writeRegister(REG_PWR_MGMT_1, 0x80)) { // Device reset
        ESP_LOGE(TAG, "Failed to send device reset to MPU9250");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Wake up and set clock source to Auto-select best available (PLL with Gyro X)
    if (!writeRegister(REG_PWR_MGMT_1, 0x01)) {
        ESP_LOGE(TAG, "Failed to wake up MPU9250");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // 4. Verify WHO_AM_I
    uint8_t who_am_i = 0;
    if (!readRegisters(REG_WHO_AM_I, &who_am_i, 1)) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return false;
    }

    ESP_LOGI(TAG, "MPU WHO_AM_I returned: 0x%02X", who_am_i);
    if (who_am_i != MPU9250_WHO_AM_I_VAL && 
        who_am_i != MPU6500_WHO_AM_I_VAL && 
        who_am_i != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Unrecognized IMU WHO_AM_I: 0x%02X (expected 0x%02X, 0x%02X, or 0x%02X)",
                 who_am_i, MPU9250_WHO_AM_I_VAL, MPU6500_WHO_AM_I_VAL, MPU6050_WHO_AM_I_VAL);
        is_healthy_ = false;
        return false;
    }

    // 5. Configure Digital Low Pass Filter (DLPF = 42 Hz)
    // CONFIG: [2:0] DLPF_CFG = 3 (42 Hz gyro bandwidth)
    if (!writeRegister(REG_CONFIG, 0x03)) return false;

    // 6. Configure Gyro Full Scale Range (±1000 dps -> FS_SEL = 2 -> 0x10)
    if (!writeRegister(REG_GYRO_CONFIG, (GYRO_FS_SEL << 3))) return false;

    // 7. Configure Accel Full Scale Range (±4g -> AFS_SEL = 1 -> 0x08)
    if (!writeRegister(REG_ACCEL_CONFIG, (ACCEL_FS_SEL << 3))) return false;

    // 8. Configure Accel DLPF (41 Hz bandwidth)
    if (!writeRegister(REG_ACCEL_CONFIG2, 0x03)) return false;

    // 9. Enable Bypass mode for I2C master so magnetometer (AK8963) can be read if needed
    if (!writeRegister(REG_INT_PIN_CFG, 0x02)) return false;

    is_healthy_ = true;
    error_count_ = 0;
    ESP_LOGI(TAG, "IMU successfully initialized and configured at 400kHz I2C");
    return true;
#else
    is_healthy_ = true;
    return true;
#endif
}

bool IMU::read(IMUData &data) {
    if (simulation_mode_) {
        data = sim_data_;
        data.valid = true;
        return true;
    }

#if defined(ESP_PLATFORM)
    uint8_t buffer[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, buffer, 14)) {
        error_count_++;
        if (error_count_ > 5) {
            is_healthy_ = false;
        }
        data.valid = false;
        return false;
    }

    // Check for I2C bus float condition (all 0x00 or all 0xFF)
    bool all_zeros = true;
    bool all_ones = true;
    for (int i = 0; i < 14; i++) {
        if (buffer[i] != 0x00) all_zeros = false;
        if (buffer[i] != 0xFF) all_ones = false;
    }
    if (all_zeros || all_ones) {
        error_count_++;
        if (error_count_ > 5) is_healthy_ = false;
        data.valid = false;
        return false;
    }

    // Assemble big-endian 16-bit signed values
    int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t raw_ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t raw_temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t raw_gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t raw_gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t raw_gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Check saturation
    if (abs(raw_ax) >= 32760 || abs(raw_ay) >= 32760 || abs(raw_az) >= 32760 ||
        abs(raw_gx) >= 32760 || abs(raw_gy) >= 32760 || abs(raw_gz) >= 32760) {
        error_count_++;
        data.valid = false;
        return false;
    }

    // Convert to engineering units
    float ax_phys = (float)raw_ax * ACCEL_SCALE_G;
    float ay_phys = (float)raw_ay * ACCEL_SCALE_G;
    float az_phys = (float)raw_az * ACCEL_SCALE_G;
    float gx_phys = (float)raw_gx * GYRO_SCALE_DPS;
    float gy_phys = (float)raw_gy * GYRO_SCALE_DPS;
    float gz_phys = (float)raw_gz * GYRO_SCALE_DPS;
    float temp_c  = ((float)raw_temp) / 333.87f + 21.0f;

    // Apply axis swap and negation from config
#if IMU_AXIS_SWAP_XY
    float tmp = ax_phys; ax_phys = ay_phys; ay_phys = tmp;
    tmp = gx_phys; gx_phys = gy_phys; gy_phys = tmp;
#endif
#if IMU_INVERT_X
    ax_phys = -ax_phys;
    gx_phys = -gx_phys;
#endif
#if IMU_INVERT_Y
    ay_phys = -ay_phys;
    gy_phys = -gy_phys;
#endif
#if IMU_INVERT_Z
    az_phys = -az_phys;
    gz_phys = -gz_phys;
#endif

    // Apply zero-bias calibration
    if (calibration_.is_calibrated) {
        ax_phys -= calibration_.ax_offset;
        ay_phys -= calibration_.ay_offset;
        az_phys -= calibration_.az_offset;
        gx_phys -= calibration_.gx_offset;
        gy_phys -= calibration_.gy_offset;
        gz_phys -= calibration_.gz_offset;
    }

    // Sanity check for NaN/Inf
    if (std::isnan(ax_phys) || std::isnan(ay_phys) || std::isnan(az_phys) ||
        std::isnan(gx_phys) || std::isnan(gy_phys) || std::isnan(gz_phys) ||
        std::isinf(ax_phys) || std::isinf(ay_phys) || std::isinf(az_phys) ||
        std::isinf(gx_phys) || std::isinf(gy_phys) || std::isinf(gz_phys)) {
        data.valid = false;
        return false;
    }

    data.ax = ax_phys;
    data.ay = ay_phys;
    data.az = az_phys;
    data.gx = gx_phys;
    data.gy = gy_phys;
    data.gz = gz_phys;
    data.temp = temp_c;
    data.valid = true;

    error_count_ = 0;
    is_healthy_ = true;
    return true;
#else
    data = sim_data_;
    data.valid = true;
    return true;
#endif
}

bool IMU::calibrate(uint16_t sample_count) {
    if (sample_count < 100) sample_count = 100;

    ESP_LOGI(TAG, "Starting IMU calibration (%d samples). Keep drone still!", sample_count);

    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;

    // Temporarily disable calibration offsets while sampling raw values
    bool prev_calibrated = calibration_.is_calibrated;
    calibration_.is_calibrated = false;

    uint16_t valid_samples = 0;
    for (uint16_t i = 0; i < sample_count; i++) {
        IMUData sample;
        if (read(sample) && sample.valid) {
            sum_ax += sample.ax;
            sum_ay += sample.ay;
            sum_az += sample.az;
            sum_gx += sample.gx;
            sum_gy += sample.gy;
            sum_gz += sample.gz;
            valid_samples++;
        }
#if defined(ESP_PLATFORM)
        vTaskDelay(pdMS_TO_TICKS(4)); // ~250 Hz sampling during calibration
#endif
    }

    if (valid_samples < (sample_count * 9 / 10)) {
        ESP_LOGE(TAG, "Calibration failed: too many dropped/invalid samples (%d/%d)",
                 valid_samples, sample_count);
        calibration_.is_calibrated = prev_calibrated;
        return false;
    }

    float mean_ax = sum_ax / (float)valid_samples;
    float mean_ay = sum_ay / (float)valid_samples;
    float mean_az = sum_az / (float)valid_samples;
    float mean_gx = sum_gx / (float)valid_samples;
    float mean_gy = sum_gy / (float)valid_samples;
    float mean_gz = sum_gz / (float)valid_samples;

    // Accel calibration assumes level flat surface:
    // ax and ay should be 0.0g, az should be +1.0g (gravity pointing down in body frame)
    calibration_.ax_offset = mean_ax;
    calibration_.ay_offset = mean_ay;
    calibration_.az_offset = mean_az - 1.0f; // Offset is difference from 1g

    calibration_.gx_offset = mean_gx;
    calibration_.gy_offset = mean_gy;
    calibration_.gz_offset = mean_gz;
    calibration_.is_calibrated = true;

    ESP_LOGI(TAG, "IMU Calibrated successfully!");
    ESP_LOGI(TAG, "Gyro offsets [dps]: X=%.3f, Y=%.3f, Z=%.3f",
             calibration_.gx_offset, calibration_.gy_offset, calibration_.gz_offset);
    ESP_LOGI(TAG, "Accel offsets [g]:   X=%.3f, Y=%.3f, Z=%.3f",
             calibration_.ax_offset, calibration_.ay_offset, calibration_.az_offset);

    return true;
}

void IMU::setSimulatedData(const IMUData &sim_data) {
    sim_data_ = sim_data;
    sim_data_.valid = true;
}

bool IMU::writeRegister(uint8_t reg, uint8_t val) {
#if defined(ESP_PLATFORM)
    uint8_t write_buf[2] = {reg, val};
    esp_err_t err = i2c_master_write_to_device(
        (i2c_port_t)I2C_PORT_NUM,
        MPU9250_I2C_ADDR,
        write_buf,
        sizeof(write_buf),
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );
    return (err == ESP_OK);
#else
    (void)reg; (void)val;
    return true;
#endif
}

bool IMU::readRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
#if defined(ESP_PLATFORM)
    esp_err_t err = i2c_master_write_read_device(
        (i2c_port_t)I2C_PORT_NUM,
        MPU9250_I2C_ADDR,
        &reg,
        1,
        buffer,
        length,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );
    return (err == ESP_OK);
#else
    (void)reg; (void)buffer; (void)length;
    return true;
#endif
}
