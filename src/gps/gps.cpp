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
        }

        if (ok) {
            GpsData fresh = {};
            fresh.latitude    = lat;
            fresh.longitude   = lon;
            fresh.speedKmh    = speed;   // SIM7000G returns km/h via AT+CGNSINF
            fresh.altitudeM   = alt;
            fresh.satellites  = (uint8_t)usat;
            fresh.valid       = true;
            fresh.timestampMs = millis();

            xSemaphoreTake(xMutexGPS, portMAX_DELAY);
            g_latestGPS = fresh;
            xSemaphoreGive(xMutexGPS);

            Serial.printf("[GPS] %.6f, %.6f  %.1f km/h  sats=%d\n",
                          lat, lon, speed, usat);
        } else {
            Serial.println("[GPS] no fix");
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
    }
}
