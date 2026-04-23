#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

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

struct OccupancyData {
    uint8_t  seatMask;        // bit N set = seat N occupied
    uint8_t  occupiedCount;
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
