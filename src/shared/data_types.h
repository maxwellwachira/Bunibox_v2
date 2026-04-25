#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include "../config.h"   // for OCC_CHIP_COUNT

// ── Core data structures ─────────────────────────────────────────────────────

struct GpsData {
    float    latitude;
    float    longitude;
    float    speedKmh;
    float    altitudeM;
    uint8_t  satellites;
    bool     valid;
    uint32_t timestampMs;
};

// Each PCF8575 is read as a 16-bit word (P00–P07 in low byte, P10–P17 in high byte).
// Bit = 1 means the sensor pulled the pin LOW → seat occupied.
// Bit = 0 means pin is HIGH (internal pull-up) → seat empty.
#define MAX_OCC_CHIPS    5   // physical maximum on the board (addresses 0x20–0x24)

struct OccupancyData {
    uint16_t chipData[MAX_OCC_CHIPS]; // one uint16 per chip; index == chip order in kOccChipAddrs
    uint8_t  chipCount;               // how many chips were successfully read this cycle
    uint8_t  occupiedCount;           // total occupied seats across all chips
    uint32_t timestampMs;
};

// ── Alert event-group bit masks ──────────────────────────────────────────────
#define ALERT_OVERSPEED    (1 << 0)
#define ALERT_CAMERA_TRIG  (1 << 1)

// ── Shared state — defined in main.cpp, extern'd everywhere else ─────────────
extern GpsData            g_latestGPS;
extern OccupancyData      g_latestOccupancy;

extern SemaphoreHandle_t  xMutexModem;
extern SemaphoreHandle_t  xMutexI2C;
extern SemaphoreHandle_t  xMutexGPS;
extern SemaphoreHandle_t  xMutexOccupancy;
extern EventGroupHandle_t xEventAlerts;
