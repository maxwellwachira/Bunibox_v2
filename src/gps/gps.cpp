#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "../shared/modem_shared.h"
#include "gps.h"

void taskGPS(void* pvParameters) {
    float lat, lon, speed, alt, accuracy;
    int   vsat, usat;
    int   year, month, day, hour, minute, sec;

    for (;;) {
        bool ok = false;

        if (xSemaphoreTake(xMutexModem, pdMS_TO_TICKS(3000))) {
            ok = modem.getGPS(&lat, &lon, &speed, &alt,
                              &vsat, &usat, &accuracy,
                              &year, &month, &day,
                              &hour, &minute, &sec);
            xSemaphoreGive(xMutexModem);
        } else {
            Serial.println("[GPS] modem busy — skipped this cycle");
        }

        if (ok) {
            GpsData fresh = {};
            fresh.latitude    = lat;
            fresh.longitude   = lon;
            fresh.speedKmh    = speed;   // SIM7000G returns km/h via AT+CGNSINF
            fresh.altitudeM   = alt;
            fresh.hdop        = accuracy;
            fresh.satellites  = (uint8_t)usat;
            fresh.visibleSats = (uint8_t)vsat;
            fresh.valid       = true;
            fresh.timestampMs = millis();
            fresh.utcYear     = (int16_t)year;
            fresh.utcMonth    = (uint8_t)month;
            fresh.utcDay      = (uint8_t)day;
            fresh.utcHour     = (uint8_t)hour;
            fresh.utcMin      = (uint8_t)minute;
            fresh.utcSec      = (uint8_t)sec;

            xSemaphoreTake(xMutexGPS, portMAX_DELAY);
            g_latestGPS = fresh;
            xSemaphoreGive(xMutexGPS);

            Serial.printf("[GPS] fix  lat=%.6f lon=%.6f  spd=%.1f km/h  alt=%.1f m\n",
                          lat, lon, speed, alt);
            Serial.printf("[GPS]      sats=%d/%d  hdop=%.1f  utc=%04d-%02d-%02d %02d:%02d:%02d\n",
                          usat, vsat, accuracy, year, month, day, hour, minute, sec);
        } else {
            // vsat > 0 means the module sees satellites but can't form a fix yet.
            // vsat == 0 typically means no signal or GPS not yet powered.
            if (vsat > 0) {
                Serial.printf("[GPS] no fix  visible=%d sats\n", vsat);
            } else {
                Serial.println("[GPS] no fix  (no satellites visible)");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
    }
}
