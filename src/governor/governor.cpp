#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "governor.h"

static bool s_fuelCut = false;

// Drive the relay and buzzer together — they always change state as a pair.
static void applyGovernor(bool cut) {
    digitalWrite(RELAY_PIN,  cut ? RELAY_ACTIVE  : !RELAY_ACTIVE);
    digitalWrite(BUZZER_PIN, cut ? HIGH           : LOW);
    s_fuelCut = cut;
}

void governorInit() {
    // Relay and buzzer outputs must be safe before any task starts.
    pinMode(RELAY_PIN,  OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    applyGovernor(false);   // fuel ON, buzzer OFF at boot
}

void taskGovernor(void* pvParameters) {
    const TickType_t kPeriod = pdMS_TO_TICKS(300);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, kPeriod);   // deterministic 300 ms cadence

        GpsData gps = {};
        xSemaphoreTake(xMutexGPS, portMAX_DELAY);
        gps = g_latestGPS;
        xSemaphoreGive(xMutexGPS);

        if (!gps.valid) continue;   // no fix — leave relay unchanged

        if (!s_fuelCut && gps.speedKmh > SPEED_LIMIT_KMH) {
            applyGovernor(true);
            xEventGroupSetBits(xEventAlerts, ALERT_OVERSPEED | ALERT_CAMERA_TRIG);
            Serial.printf("[GOV] OVERSPEED %.1f km/h — fuel CUT, buzzer ON\n", gps.speedKmh);

        } else if (s_fuelCut && gps.speedKmh < SPEED_RESTORE_KMH) {
            applyGovernor(false);
            xEventGroupClearBits(xEventAlerts, ALERT_OVERSPEED);
            Serial.printf("[GOV] speed safe %.1f km/h — fuel RESTORED, buzzer OFF\n", gps.speedKmh);
        }
    }
}
