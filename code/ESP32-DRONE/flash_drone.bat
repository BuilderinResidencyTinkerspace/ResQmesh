@echo off
setlocal enabledelayedexpansion
title XIAO ESP32-S3 Drone Firmware Flasher

echo ===================================================================
echo   AeroCommand // Seeed XIAO ESP32-S3 Drone Firmware Flasher
echo ===================================================================
echo.

set "CLI_PATH=C:\Users\jayas\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set "SKETCH_DIR=%~dp0firmware"
set "TMP_BOARDS=%TEMP%\arduino_boards_%RANDOM%.tmp"

:: 1. Check if CLI exists
if not exist "%CLI_PATH%" (
    echo [ERROR] Arduino CLI executable not found at:
    echo   "%CLI_PATH%"
    pause
    exit /b 1
)

:: 2. Auto-detect COM ports
echo [*] Scanning available USB Serial / COM ports...
echo.
echo -------------------------------------------------------------------
"%CLI_PATH%" board list > "%TMP_BOARDS%" 2>nul
type "%TMP_BOARDS%"
echo -------------------------------------------------------------------
echo.

set "CACHE_FILE=%~dp0.last_com_port"
set "LAST_PORT=COM14"
if exist "%CACHE_FILE%" set /p LAST_PORT=<"%CACHE_FILE%"

set DETECTED_PORT=
:: First check arduino-cli board list
for /f "tokens=1,2,*" %%A in ('findstr /i "ESP32" "%TMP_BOARDS%" 2^>nul') do (
    if "!DETECTED_PORT!"=="" set DETECTED_PORT=%%A
)
if exist "%TMP_BOARDS%" del "%TMP_BOARDS%" 2>nul

:: If not found, check registry for non-Bluetooth USB Serial ports
if "!DETECTED_PORT!"=="" (
    for /f "tokens=3" %%P in ('reg query "HKLM\HARDWARE\DEVICEMAP\SERIALCOMM" 2^>nul ^| findstr /v /i "BthModem"') do (
        if "!DETECTED_PORT!"=="" set "DETECTED_PORT=%%P"
    )
)

:: 3. Select COM Port
set TARGET_PORT=%~1

if "!TARGET_PORT!"=="" (
    if not "!DETECTED_PORT!"=="" (
        echo [OK] Auto-detected USB device on: !DETECTED_PORT!
        set "USER_INPUT="
        set /p USER_INPUT="Enter COM port [Press ENTER to use !DETECTED_PORT!]: "
        if "!USER_INPUT!"=="" (
            set TARGET_PORT=!DETECTED_PORT!
        ) else (
            set TARGET_PORT=!USER_INPUT!
        )
    ) else (
        echo [!] No active USB serial port detected - ensure drone is plugged in via USB-C.
        set "USER_INPUT="
        set /p USER_INPUT="Enter COM port [Press ENTER for !LAST_PORT!]: "
        if "!USER_INPUT!"=="" (
            set TARGET_PORT=!LAST_PORT!
        ) else (
            set TARGET_PORT=!USER_INPUT!
        )
    )
)

:: Trim spaces
if not "!TARGET_PORT!"=="" (
    for /f "tokens=1" %%x in ("!TARGET_PORT!") do set TARGET_PORT=%%x
)

:: Normalize port name (e.g. 14 -> COM14)
if not "!TARGET_PORT!"=="" (
    if /i not "!TARGET_PORT:~0,3!"=="COM" (
        set TARGET_PORT=COM!TARGET_PORT!
    )
)

if "!TARGET_PORT!"=="" (
    echo [ERROR] No COM port specified. Aborting.
    pause
    exit /b 1
)

echo.
echo [*] Target Upload Port: !TARGET_PORT!
echo.

:: 4. Compile Firmware
echo [1/2] Compiling firmware for XIAO ESP32-S3...
"%CLI_PATH%" compile --fqbn esp32:esp32:XIAO_ESP32S3 "%SKETCH_DIR%"
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Compilation failed!
    pause
    exit /b %ERRORLEVEL%
)

:: 5. Upload Firmware
:UPLOAD_STEP
echo.
echo [2/2] Uploading firmware to !TARGET_PORT!...
"%CLI_PATH%" upload -p !TARGET_PORT! --fqbn esp32:esp32:XIAO_ESP32S3 "%SKETCH_DIR%"
if %ERRORLEVEL% EQU 0 goto UPLOAD_SUCCESS

echo.
echo ===================================================================
echo   [UPLOAD FAILED on !TARGET_PORT!]
echo ===================================================================
echo   Troubleshooting steps:
echo     1. Port Busy / Access Denied:
echo        - Close any open Serial Monitor (Arduino IDE, PuTTY, etc.)
echo        - Disconnect or close WebSerial in the AeroCommand browser tab
echo        - Close any other terminal running flash_drone.bat
echo     2. Wrong Port:
echo        - Check the detected board list above to confirm port
echo     3. Bootloader Mode:
echo        - Hold the 'B' (Boot) button on your Seeed XIAO ESP32-S3
echo        - Press and release the 'R' (Reset) button
echo        - Release the 'B' button, then retry upload
echo ===================================================================
echo.
set RETRY=
set /p RETRY="Would you like to try another COM port or retry? (Y/N, default Y): "
if "!RETRY!"=="" set RETRY=Y
if /i "!RETRY!"=="Y" (
    set /p NEW_PORT="Enter COM port (press ENTER to retry !TARGET_PORT!): "
    if not "!NEW_PORT!"=="" (
        if /i not "!NEW_PORT:~0,3!"=="COM" set NEW_PORT=COM!NEW_PORT!
        set TARGET_PORT=!NEW_PORT!
    )
    goto UPLOAD_STEP
)
pause
exit /b 1

:UPLOAD_SUCCESS
echo !TARGET_PORT!> "%CACHE_FILE%" 2>nul
echo.
echo ===================================================================
echo   FLASHING PROCESS FINISHED SUCCESSFULLY!
echo ===================================================================
echo   Firmware is now running on your Seeed XIAO ESP32-S3 (!TARGET_PORT!).
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

set OPEN_MON=
set /p OPEN_MON="Open USB Serial Monitor now? (Y/N, default N): "
if /i "!OPEN_MON!"=="Y" (
    echo Starting Serial Monitor at 115200 baud on !TARGET_PORT!...
    echo (Press Ctrl+C to exit monitor)
    "%CLI_PATH%" monitor -p !TARGET_PORT! -c baudrate=115200
)
