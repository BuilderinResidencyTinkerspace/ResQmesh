/**
 * @file battery.h
 * @brief 1S LiPo Battery Monitor with ADC Calibration and Low-Voltage Alarm.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

class BatteryMonitor {
public:
    BatteryMonitor(float divider_ratio = BATTERY_DIVIDER_RATIO);
    ~BatteryMonitor();

    /**
     * @brief Configure ADC channel, attenuation, and calibration characteristics.
     * @return true if ADC initialized successfully.
     */
    bool init();

    /**
     * @brief Sample ADC, apply moving average filter, and compute battery voltage.
     * @return Filtered battery voltage in Volts.
     */
    float readVoltage();

    /**
     * @brief Get cached filtered battery voltage.
     */
    float getVoltage() const { return filtered_voltage_; }

    /**
     * @brief Calculate remaining battery capacity percentage (0 to 100%).
     */
    float getPercentage() const;

    /**
     * @brief True if battery is below warning threshold (BATTERY_WARN_VOLTS).
     */
    bool isLow() const { return (filtered_voltage_ > 1.0f && filtered_voltage_ < BATTERY_WARN_VOLTS); }

    /**
     * @brief True if battery is below critical threshold (BATTERY_CRIT_VOLTS).
     */
    bool isCritical() const { return (filtered_voltage_ > 1.0f && filtered_voltage_ < BATTERY_CRIT_VOLTS); }

    /**
     * @brief Inject simulated battery voltage for testing and simulation.
     */
    void setSimulatedVoltage(float volts) {
        simulated_voltage_ = volts;
        use_simulation_ = true;
    }

private:
    float divider_ratio_;
    float filtered_voltage_;
    float sample_buffer_[BATTERY_ADC_SAMPLES];
    uint8_t sample_idx_;
    bool buffer_filled_;
    bool use_simulation_;
    float simulated_voltage_;

#if defined(ESP_PLATFORM)
    void *adc_handle_;
#endif
};
