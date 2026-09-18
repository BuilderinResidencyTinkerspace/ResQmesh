/**
 * @file cli.cpp
 * @brief Interactive Serial Debugging CLI Implementation.
 */

#include "cli.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

#if defined(ARDUINO)
#include <Arduino.h>
#define CLI_PRINTF(...) Serial.printf(__VA_ARGS__)
#define CLI_FLUSH()     Serial.flush()
#else
#define CLI_PRINTF(...) printf(__VA_ARGS__)
#define CLI_FLUSH()     fflush(stdout)
#endif

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static const char *TAG = "CLI";
#endif

SerialCLI::SerialCLI(FlightController &fc)
    : fc_(fc),
      line_idx_(0),
      streaming_enabled_(false),
      last_stream_ms_(0) {
    line_buf_[0] = '\0';
}

bool SerialCLI::start() {
#if defined(ARDUINO)
    CLI_PRINTF("\r\n=======================================================\r\n");
    CLI_PRINTF("  %s v%s - Flight Controller Console Ready\r\n", FIRMWARE_NAME, FIRMWARE_VERSION);
    CLI_PRINTF("  Target: %s\r\n", HARDWARE_TARGET);
    CLI_PRINTF("  Type 'help' for available commands.\r\n");
    CLI_PRINTF("=======================================================\r\n\r\n> ");
    CLI_FLUSH();
    return true;
#elif defined(ESP_PLATFORM)
    BaseType_t ret = xTaskCreatePinnedToCore(
        cliTask,
        "cli_task",
        CLI_TASK_STACK_SIZE,
        this,
        CLI_TASK_PRIO,
        nullptr,
        CLI_TASK_CORE
    );
    return (ret == pdPASS);
#else
    return true;
#endif
}

void SerialCLI::update() {
#if defined(ARDUINO)
    uint32_t now = millis();
    if (streaming_enabled_ && (now - last_stream_ms_ >= 50)) { // 20 Hz
        last_stream_ms_ = now;
        const IMUData &d = fc_.getLatestIMUData();
        const LoopStats &stats = fc_.getStats();
        CLI_PRINTF("$TELEM,%s,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.0f,%.1f,%d\r\n",
                   fc_.getSafety().getStateString(),
                   fc_.getAttitude().getRoll(),
                   fc_.getAttitude().getPitch(),
                   fc_.getAttitude().getYawRate(),
                   d.ax, d.ay, d.az,
                   d.gx, d.gy, d.gz,
                   (unsigned)fc_.getMotors().getCommandedDuty(0),
                   (unsigned)fc_.getMotors().getCommandedDuty(1),
                   (unsigned)fc_.getMotors().getCommandedDuty(2),
                   (unsigned)fc_.getMotors().getCommandedDuty(3),
                   fc_.getBattery().getVoltage(),
                   fc_.getBattery().getPercentage(),
                   stats.current_freq_hz,
                   fc_.getSafety().isArmed() ? 1 : 0);
    }

    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (line_idx_ > 0) {
                line_buf_[line_idx_] = '\0';
                Serial.println();
                processCommand(line_buf_);
                line_idx_ = 0;
                if (!streaming_enabled_) {
                    Serial.print("\r\n> ");
                }
                Serial.flush();
            }
        } else if (c == 0x08 || c == 0x7F) { // Backspace
            if (line_idx_ > 0) {
                line_idx_--;
                Serial.print("\b \b");
            }
        } else if (line_idx_ < (int)sizeof(line_buf_) - 1) {
            line_buf_[line_idx_++] = (char)c;
            if (!streaming_enabled_) {
                Serial.write((char)c); // echo back to terminal
            }
        }
    }
#endif
}

void SerialCLI::cliTask(void *arg) {
    SerialCLI *cli = reinterpret_cast<SerialCLI*>(arg);
    char line_buf[128];
    int idx = 0;

    CLI_PRINTF("\r\n=======================================================\r\n");
    CLI_PRINTF("  %s v%s - Flight Controller Console Ready\r\n", FIRMWARE_NAME, FIRMWARE_VERSION);
    CLI_PRINTF("  Target: %s\r\n", HARDWARE_TARGET);
    CLI_PRINTF("  Type 'help' for available commands.\r\n");
    CLI_PRINTF("=======================================================\r\n\r\n> ");
    CLI_FLUSH();

    while (true) {
        int c = getchar();
        if (c != EOF && c > 0) {
            if (c == '\r' || c == '\n') {
                if (idx > 0) {
                    line_buf[idx] = '\0';
                    CLI_PRINTF("\r\n");
                    cli->processCommand(line_buf);
                    idx = 0;
                    CLI_PRINTF("\r\n> ");
                    CLI_FLUSH();
                }
            } else if (c == 0x08 || c == 0x7F) { // Backspace
                if (idx > 0) {
                    idx--;
                    CLI_PRINTF("\b \b");
                    CLI_FLUSH();
                }
            } else if (idx < (int)sizeof(line_buf) - 1) {
                line_buf[idx++] = (char)c;
                putchar(c);
                CLI_FLUSH();
            }
        } else {
#if defined(ESP_PLATFORM)
            vTaskDelay(pdMS_TO_TICKS(20)); // Yield CPU
#endif
        }
    }
}

void SerialCLI::processCommand(const char *cmd_line) {
    // Skip leading spaces
    while (*cmd_line == ' ') cmd_line++;
    if (*cmd_line == '\0') return;

    if (strncmp(cmd_line, "status", 6) == 0) {
        printStatus();
    } else if (strncmp(cmd_line, "imu", 3) == 0) {
        printIMU();
    } else if (strncmp(cmd_line, "attitude", 8) == 0) {
        printAttitude();
    } else if (strncmp(cmd_line, "pid", 3) == 0) {
        printPID();
    } else if (strncmp(cmd_line, "battery", 7) == 0) {
        printBattery();
    } else if (strncmp(cmd_line, "motors", 6) == 0) {
        printMotors();
    } else if (strncmp(cmd_line, "calibrate", 9) == 0) {
        CLI_PRINTF("Initiating IMU calibration...\r\n");
        if (fc_.calibrateSensors()) {
            CLI_PRINTF("SUCCESS: IMU calibration complete.\r\n");
        } else {
            CLI_PRINTF("FAILED: Could not calibrate. Ensure drone is motionless and disarmed.\r\n");
        }
    } else if (strncmp(cmd_line, "arm", 3) == 0) {
        CLI_PRINTF("Software arm requested...\r\n");
        if (fc_.getSafety().isArmed()) {
            CLI_PRINTF("Already armed.\r\n");
        } else {
            CLI_PRINTF("NOTE: Drone must be armed via transmitter for active flight safety.\r\n");
        }
    } else if (strncmp(cmd_line, "disarm", 6) == 0) {
        fc_.requestDisarm();
        CLI_PRINTF("Disarmed. Motors stopped.\r\n");
    } else if (strncmp(cmd_line, "test_motor", 10) == 0) {
        handleMotorTest(cmd_line + 10);
    } else if (strncmp(cmd_line, "sim", 3) == 0) {
        handleSimulate(cmd_line + 3);
    } else if (strncmp(cmd_line, "stream", 6) == 0) {
        if (strstr(cmd_line, "on") != nullptr) {
            streaming_enabled_ = true;
            CLI_PRINTF("Telemetry streaming ENABLED (20 Hz)\r\n");
        } else {
            streaming_enabled_ = false;
            CLI_PRINTF("Telemetry streaming DISABLED\r\n");
        }
    } else if (strncmp(cmd_line, "help", 4) == 0) {
        printHelp();
    } else {
        CLI_PRINTF("Unknown command: '%s'. Type 'help' for commands.\r\n", cmd_line);
    }
}

void SerialCLI::printStatus() {
    const LoopStats &stats = fc_.getStats();
    float roll = fc_.getAttitude().getRoll();
    float pitch = fc_.getAttitude().getPitch();
    float yaw_rate = fc_.getAttitude().getYawRate();
    float batt_v = fc_.getBattery().getVoltage();
    float batt_pct = fc_.getBattery().getPercentage();

    CLI_PRINTF("\r\n## STATUS\r\n");
    CLI_PRINTF("State:       %s\r\n", fc_.getSafety().getStateString());
    CLI_PRINTF("IMU:         %s (Calibrated: %s)\r\n", 
           fc_.getIMU().isHealthy() ? "OK" : "ERROR",
           fc_.getIMU().isCalibrated() ? "YES" : "NO");
    CLI_PRINTF("Receiver:    %s (Packets: %u, Lost: %u)\r\n",
           fc_.getReceiver().isConnected() ? "OK" : "DISCONNECTED",
           (unsigned)fc_.getLatestReceiverData().packets_received,
           (unsigned)fc_.getLatestReceiverData().packets_lost);
    CLI_PRINTF("Battery:     %.2f V (%.0f%%) [%s]\r\n",
           batt_v, batt_pct,
           fc_.getBattery().isCritical() ? "CRITICAL" : (fc_.getBattery().isLow() ? "LOW" : "OK"));
    CLI_PRINTF("Loop:        %.1f Hz (Total: %u us, IMU: %u us, Compute: %u us, Overruns: %u)\r\n",
           stats.current_freq_hz, stats.loop_time_us, stats.imu_read_time_us,
           stats.compute_time_us, stats.overruns);
    CLI_PRINTF("Roll:        %+.2f deg\r\n", roll);
    CLI_PRINTF("Pitch:       %+.2f deg\r\n", pitch);
    CLI_PRINTF("YawRate:     %+.2f deg/s\r\n", yaw_rate);
    CLI_PRINTF("Motors:      M1=%u M2=%u M3=%u M4=%u\r\n",
           (unsigned)fc_.getMotors().getCommandedDuty(0),
           (unsigned)fc_.getMotors().getCommandedDuty(1),
           (unsigned)fc_.getMotors().getCommandedDuty(2),
           (unsigned)fc_.getMotors().getCommandedDuty(3));
}

void SerialCLI::printIMU() {
    const IMUData &d = fc_.getLatestIMUData();
    const IMUCalibration &cal = fc_.getIMU().getCalibration();
    CLI_PRINTF("\r\n## IMU DATA\r\n");
    CLI_PRINTF("Accel [g]:     X=%+.3f, Y=%+.3f, Z=%+.3f\r\n", d.ax, d.ay, d.az);
    CLI_PRINTF("Gyro  [deg/s]: X=%+.3f, Y=%+.3f, Z=%+.3f\r\n", d.gx, d.gy, d.gz);
    CLI_PRINTF("Temperature:   %.1f C\r\n", d.temp);
    CLI_PRINTF("Gyro Offsets:  X=%+.3f, Y=%+.3f, Z=%+.3f\r\n", cal.gx_offset, cal.gy_offset, cal.gz_offset);
    CLI_PRINTF("Accel Offsets: X=%+.3f, Y=%+.3f, Z=%+.3f\r\n", cal.ax_offset, cal.ay_offset, cal.az_offset);
}

void SerialCLI::printAttitude() {
    CLI_PRINTF("\r\n## ATTITUDE ESTIMATOR\r\n");
    CLI_PRINTF("Roll:      %+.2f deg\r\n", fc_.getAttitude().getRoll());
    CLI_PRINTF("Pitch:     %+.2f deg\r\n", fc_.getAttitude().getPitch());
    CLI_PRINTF("RollRate:  %+.2f deg/s\r\n", fc_.getAttitude().getRollRate());
    CLI_PRINTF("PitchRate: %+.2f deg/s\r\n", fc_.getAttitude().getPitchRate());
    CLI_PRINTF("YawRate:   %+.2f deg/s\r\n", fc_.getAttitude().getYawRate());
    CLI_PRINTF("Alpha:     %.4f\r\n", fc_.getAttitude().getAlpha());
}

void SerialCLI::printPID() {
    const PIDConfig &r_ang = fc_.getRollController().getAnglePID().getConfig();
    const PIDConfig &r_rat = fc_.getRollController().getRatePID().getConfig();
    const PIDConfig &y_rat = fc_.getYawController().getConfig();

    CLI_PRINTF("\r\n## PID CONTROLLER CONFIGURATION\r\n");
    CLI_PRINTF("Roll Angle Loop:  Kp=%.3f, Ki=%.3f, Kd=%.3f, MaxOut=%.1f\r\n",
           r_ang.kp, r_ang.ki, r_ang.kd, r_ang.max_output);
    CLI_PRINTF("Roll Rate Loop:   Kp=%.3f, Ki=%.3f, Kd=%.3f, MaxInt=%.1f, MaxOut=%.1f\r\n",
           r_rat.kp, r_rat.ki, r_rat.kd, r_rat.max_integral, r_rat.max_output);
    CLI_PRINTF("Yaw Rate Loop:    Kp=%.3f, Ki=%.3f, Kd=%.3f, MaxInt=%.1f, MaxOut=%.1f\r\n",
           y_rat.kp, y_rat.ki, y_rat.kd, y_rat.max_integral, y_rat.max_output);
    CLI_PRINTF("Last Errors:      RollRateErr=%+.2f, PitchRateErr=%+.2f, YawRateErr=%+.2f\r\n",
           fc_.getRollController().getRatePID().getLastError(),
           fc_.getPitchController().getRatePID().getLastError(),
           fc_.getYawController().getLastError());
}

void SerialCLI::printBattery() {
    CLI_PRINTF("\r\n## BATTERY MONITOR\r\n");
    CLI_PRINTF("Voltage:      %.3f V\r\n", fc_.getBattery().getVoltage());
    CLI_PRINTF("Percentage:   %.1f %%\r\n", fc_.getBattery().getPercentage());
    CLI_PRINTF("Low Alert:    %s (Threshold: %.2f V)\r\n",
           fc_.getBattery().isLow() ? "YES" : "NO", BATTERY_WARN_VOLTS);
    CLI_PRINTF("Crit Alert:   %s (Threshold: %.2f V)\r\n",
           fc_.getBattery().isCritical() ? "YES" : "NO", BATTERY_CRIT_VOLTS);
}

void SerialCLI::printMotors() {
    CLI_PRINTF("\r\n## MOTOR OUTPUTS (PWM Duty 0-1023)\r\n");
    CLI_PRINTF("M1 (Front-Left,  CW):  %u\r\n", (unsigned)fc_.getMotors().getCommandedDuty(0));
    CLI_PRINTF("M2 (Front-Right, CCW): %u\r\n", (unsigned)fc_.getMotors().getCommandedDuty(1));
    CLI_PRINTF("M3 (Rear-Right,  CW):  %u\r\n", (unsigned)fc_.getMotors().getCommandedDuty(2));
    CLI_PRINTF("M4 (Rear-Left,   CCW): %u\r\n", (unsigned)fc_.getMotors().getCommandedDuty(3));
    CLI_PRINTF("Armed State: %s\r\n", fc_.getMotors().isArmed() ? "ARMED" : "DISARMED");
}

void SerialCLI::handleMotorTest(const char *args) {
#if !MOTOR_TEST_ENABLED
    CLI_PRINTF("ERROR: Motor test disabled at compile time (#define MOTOR_TEST_ENABLED 0 in config.h)\r\n");
    CLI_PRINTF("To enable: change MOTOR_TEST_ENABLED to 1 in config.h and re-flash.\r\n");
#else
    int motor_id = 0;
    float duty_pct = 0.0f;
    if (sscanf(args, "%d %f", &motor_id, &duty_pct) != 2) {
        CLI_PRINTF("Usage: test_motor <motor_id 1-4> <duty_percent 0-25>\r\n");
        CLI_PRINTF("WARNING: REMOVE PROPELLERS BEFORE BENCH TESTING MOTORS!\r\n");
        return;
    }

    if (motor_id < 1 || motor_id > 4) {
        CLI_PRINTF("Invalid motor ID. Must be 1 to 4.\r\n");
        return;
    }

    if (fc_.getMotors().testMotor((uint8_t)(motor_id - 1), duty_pct)) {
        CLI_PRINTF("Motor M%d pulsed at %.1f%% duty.\r\n", motor_id, duty_pct);
    } else {
        CLI_PRINTF("Motor test rejected by safety supervisor.\r\n");
    }
#endif
}

void SerialCLI::handleSimulate(const char *args) {
    float sim_roll = 0.0f;
    float sim_pitch = 0.0f;
    if (sscanf(args, "%f %f", &sim_roll, &sim_pitch) != 2) {
        CLI_PRINTF("Usage: sim <roll_deg> <pitch_deg>\r\n");
        return;
    }

    IMUData sim_imu = {};
    sim_imu.ax = -sinf(sim_pitch * 3.14159f / 180.0f);
    sim_imu.ay = sinf(sim_roll * 3.14159f / 180.0f);
    sim_imu.az = cosf(sim_roll * 3.14159f / 180.0f) * cosf(sim_pitch * 3.14159f / 180.0f);
    sim_imu.valid = true;

    fc_.getIMU().enableSimulation(true);
    fc_.getIMU().setSimulatedData(sim_imu);
    CLI_PRINTF("Injected simulated IMU: Roll=%.1f deg, Pitch=%.1f deg\r\n", sim_roll, sim_pitch);
}

void SerialCLI::printHelp() {
    CLI_PRINTF("\r\nAvailable Commands:\r\n");
    CLI_PRINTF("  status        - Overview of state, sensors, battery, and loop statistics\r\n");
    CLI_PRINTF("  imu           - Real-time raw and calibrated IMU accelerometer and gyro data\r\n");
    CLI_PRINTF("  attitude      - Complementary filter roll, pitch, and yaw-rate estimates\r\n");
    CLI_PRINTF("  pid           - Current cascaded PID controller gains and error tracking\r\n");
    CLI_PRINTF("  battery       - 1S LiPo battery voltage, percentage, and alarm thresholds\r\n");
    CLI_PRINTF("  motors        - Current PWM duty cycles commanded to all 4 motors\r\n");
    CLI_PRINTF("  calibrate     - Trigger zero-bias sensor calibration (must be motionless & disarmed)\r\n");
    CLI_PRINTF("  disarm        - Force emergency disarm and cut motor power\r\n");
    CLI_PRINTF("  test_motor    - Bench pulse motor: test_motor <1-4> <0-25%%> (requires compile flag)\r\n");
    CLI_PRINTF("  sim           - Inject virtual orientation: sim <roll_deg> <pitch_deg>\r\n");
    CLI_PRINTF("  help          - Display this help message\r\n");
}
