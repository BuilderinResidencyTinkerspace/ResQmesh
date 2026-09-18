@echo off
echo ========================================================
echo   Compiling & Running ESP32-DRONE Flight Simulation
echo ========================================================

g++ -std=c++11 -Wall -I../main simulate_flight.cpp ../main/flight_controller.cpp ../main/motors.cpp ../main/imu.cpp ../main/attitude.cpp ../main/pid.cpp ../main/mixer.cpp ../main/receiver.cpp ../main/battery.cpp ../main/safety.cpp -o simulate_flight.exe

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Simulation compilation failed!
    exit /b %ERRORLEVEL%
)

simulate_flight.exe
