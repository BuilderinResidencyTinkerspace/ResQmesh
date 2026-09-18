/**
 * @file web_ui.h
 * @brief Zero-Install Mobile Touch Flight Controller UI for ESP32-S3 Drone.
 * Stored in flash (PROGMEM) and served over HTTP to smartphones.
 */

#pragma once

#if defined(ESP_PLATFORM)
#include <esp_attr.h>
#define PROGMEM
#else
#define PROGMEM
#endif

#include "firmware/web_ui.h"
