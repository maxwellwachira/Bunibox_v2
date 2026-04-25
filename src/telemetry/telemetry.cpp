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

    // GPS / speed
    doc["ts"]        = gps.timestampMs;
    doc["lat"]       = serialized(String(gps.latitude,  6));
    doc["lng"]       = serialized(String(gps.longitude, 6));
    doc["speed"]     = gps.speedKmh;
    doc["alt"]       = gps.altitudeM;
    doc["sats"]      = gps.satellites;
    doc["gps_ok"]    = gps.valid;
    doc["overspeed"] = overspeed;

    // Seat occupancy — one uint16 per PCF8575 chip, sent as a JSON array.
    // Bit N of chipData[i] = 1 means the (N+1)th seat on chip i is occupied.
    // The server maps chip index + bit position to a physical seat number.
    doc["occupied"] = occ.occupiedCount;
    JsonArray chips = doc["seat_chips"].to<JsonArray>();
    for (int i = 0; i < occ.chipCount; i++) {
        chips.add(occ.chipData[i]);
    }

    String out;
    serializeJson(doc, out);
    return out;
}

void taskTelemetry(void* pvParameters) {
    static TinyGsmClient gsmClient(modem, 0);

    const TickType_t kInterval = pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS);
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, kInterval);

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
