#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "governor.h"

static bool s_fuelCut = false;

static void setRelay(bool cut) {
    digitalWrite(RELAY_PIN, cut ? RELAY_ACTIVE : !RELAY_ACTIVE);
    s_fuelCut = cut;
}

void governorInit() {
    pinMode(RELAY_PIN, OUTPUT);
    setRelay(false);   // safe state: fuel ON at boot
}

void taskGovernor(void* pvParameters) {
    const TickType_t kPeriod = pdMS_TO_TICKS(300);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, kPeriod);  // deterministic 300 ms loop

        GpsData gps = {};
        xSemaphoreTake(xMutexGPS, portMAX_DELAY);
        gps = g_latestGPS;
        xSemaphoreGive(xMutexGPS);

        if (!gps.valid) continue;

        if (!s_fuelCut && gps.speedKmh > SPEED_LIMIT_KMH) {
            setRelay(true);
            xEventGroupSetBits(xEventAlerts, ALERT_OVERSPEED | ALERT_CAMERA_TRIG);
            Serial.printf("[GOV] OVERSPEED %.1f km/h — fuel CUT\n", gps.speedKmh);

        } else if (s_fuelCut && gps.speedKmh < SPEED_RESTORE_KMH) {
            setRelay(false);
            xEventGroupClearBits(xEventAlerts, ALERT_OVERSPEED);
            Serial.printf("[GOV] speed safe %.1f km/h — fuel RESTORED\n", gps.speedKmh);
        }
    }
}
