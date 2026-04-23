#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "../shared/modem_shared.h"
#include "telemetry.h"

static bool ensureGprs() {
    if (modem.isGprsConnected()) return true;
    Serial.println("[TEL] GPRS dropped — reconnecting...");
    return modem.gprsConnect(CELL_APN, "", "");
}

static String buildPayload(const GpsData& gps,
                           const OccupancyData& occ,
                           bool overspeed) {
    JsonDocument doc;
    doc["ts"]        = gps.timestampMs;
    doc["lat"]       = gps.latitude;
    doc["lng"]       = gps.longitude;
    doc["speed"]     = gps.speedKmh;
    doc["alt"]       = gps.altitudeM;
    doc["sats"]      = gps.satellites;
    doc["gps_ok"]    = gps.valid;
    doc["seats"]     = occ.seatMask;
    doc["occupied"]  = occ.occupiedCount;
    doc["overspeed"] = overspeed;

    String out;
    serializeJson(doc, out);
    return out;
}

void taskTelemetry(void* pvParameters) {
    // Created on first task entry — modem is fully initialised by this point
    static TinyGsmClient gsmClient(modem, 0);

    const TickType_t kInterval = pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, kInterval);

        // Snapshot shared state
        GpsData gps = {};
        xSemaphoreTake(xMutexGPS, portMAX_DELAY);
        gps = g_latestGPS;
        xSemaphoreGive(xMutexGPS);

        if (!gps.valid) {
            Serial.println("[TEL] skip — no GPS fix");
            continue;
        }

        OccupancyData occ = {};
        xSemaphoreTake(xMutexOccupancy, portMAX_DELAY);
        occ = g_latestOccupancy;
        xSemaphoreGive(xMutexOccupancy);

        bool overspeed = (xEventGroupGetBits(xEventAlerts) & ALERT_OVERSPEED) != 0;
        String payload = buildPayload(gps, occ, overspeed);

        if (!xSemaphoreTake(xMutexModem, pdMS_TO_TICKS(5000))) {
            Serial.println("[TEL] modem busy — skip");
            continue;
        }

        if (ensureGprs()) {
            HttpClient http(gsmClient, SERVER_HOST, SERVER_PORT);
            int err = http.post(SERVER_PATH, "application/json", payload);
            if (err == HTTP_SUCCESS) {
                Serial.printf("[TEL] POST %d  (%d bytes)\n",
                              http.responseStatusCode(), payload.length());
            } else {
                Serial.printf("[TEL] HTTP error %d\n", err);
            }
            http.stop();
        }

        xSemaphoreGive(xMutexModem);
    }
}
