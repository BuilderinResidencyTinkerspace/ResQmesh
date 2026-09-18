@echo off
echo ========================================================
echo   Compiling & Running ESP32-DRONE Native Unit Tests
echo ========================================================

g++ -std=c++11 -Wall -I../main run_all_tests.cpp ../main/pid.cpp ../main/attitude.cpp ../main/mixer.cpp ../main/receiver.cpp ../main/battery.cpp ../main/safety.cpp ../main/imu.cpp -o test_runner.exe

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation failed!
    exit /b %ERRORLEVEL%
)

test_runner.exe
