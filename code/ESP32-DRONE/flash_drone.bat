@echo off
setlocal
echo ===================================================================
echo   Flashing Custom Drone Flight Controller to XIAO ESP32-S3 (COM10)
echo ===================================================================

set CLI_PATH="C:\Users\jayas\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set SKETCH_DIR="%~dp0firmware"

echo [1/2] Compiling firmware...
%CLI_PATH% compile --fqbn esp32:esp32:XIAO_ESP32S3 %SKETCH_DIR%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation failed!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [2/2] Uploading firmware to COM10...
%CLI_PATH% upload -p COM10 --fqbn esp32:esp32:XIAO_ESP32S3 %SKETCH_DIR%

echo.
echo ===================================================================
echo   FLASHING PROCESS FINISHED!
echo ===================================================================
echo   If you saw "Hash of data verified" and "Hard resetting with RTC WDT":
echo   --> The firmware was SUCCESSFUL and is now running on your drone!
echo.
echo   [Option A: Fly via Smartphone Wi-Fi]
echo     1. Open Wi-Fi on your phone and connect to: ResQmesh-Drone
echo     2. Password:                                12345678
echo     3. Open browser and navigate to:            http://192.168.4.1
echo.
echo   [Option B: Desktop Ground Control Station]
echo     Run: .\code\ESP32-DRONE\open_gui.bat
echo ===================================================================
echo.
set /p OPEN_MON="Open USB Serial Monitor in this terminal? (Y/N, default N): "
if /i "%OPEN_MON%"=="Y" (
    %CLI_PATH% monitor -p COM10 -c baudrate=115200
)
