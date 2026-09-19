/**
 * @file cli.h
 * @brief Interactive Serial Debugging and Diagnostic CLI Interface.
 */

#pragma once

#include "flight_controller.h"

class SerialCLI {
public:
    SerialCLI(FlightController &fc);

    /**
     * @brief Start the background CLI task on Core 0.
     */
    bool start();

    /**
     * @brief Polls USB Serial for incoming command characters (for Arduino loop).
     */
    void update();

    /**
     * @brief Process an individual command line string.
     * Accessible for testing or automated script feeds.
     */
    void processCommand(const char *cmd_line);

private:
    static void cliTask(void *arg);

    void printStatus();
    void printIMU();
    void printAttitude();
    void printPID();
    void printBattery();
    void printMotors();
    void printSwarm();
    void printHelp();
    void handleNodeId(const char *args);
    void handleRole(const char *args);
    void handleSwarmCmd(const char *args);
    void handleMotorTest(const char *args);
    void handleSimulate(const char *args);

    FlightController &fc_;
    char line_buf_[128];
    int line_idx_;
    bool streaming_enabled_;
    uint32_t last_stream_ms_;
};
