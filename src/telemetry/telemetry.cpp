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

// SIM7000 RSSI: 0=≤-113 dBm, 1-30=-111 to -53 dBm, 31=≥-51 dBm, 99=unknown.
static String rssiLabel(int rssi) {
    if (rssi == 99) return "no signal";
    if (rssi == 0)  return "≤-113 dBm";
    if (rssi == 31) return "≥-51 dBm";
    return String(-113 + 2 * rssi) + " dBm";
}

static bool ensureGprs() {
    if (modem.isGprsConnected()) return true;
    Serial.printf("[TEL] GPRS dropped (%s) — reconnecting...\n",
                  rssiLabel(modem.getSignalQuality()).c_str());
    bool ok = modem.gprsConnect(CELL_APN, "", "");
    if (ok) {
        Serial.printf("[TEL] GPRS up  operator=%s  signal=%s  IP=%s\n",
                      modem.getOperator().c_str(),
                      rssiLabel(modem.getSignalQuality()).c_str(),
                      modem.getLocalIP().c_str());
    } else {
        Serial.printf("[TEL] GPRS reconnect failed  signal=%s\n",
                      rssiLabel(modem.getSignalQuality()).c_str());
    }
    return ok;
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

        // Check / restore GPRS — this can block for up to 60 s during reconnect.
        // Hold the mutex only for this operation, release before HTTP so GPS
        // isn't starved across the entire reconnect window.
        bool gprsUp = false;
        if (xSemaphoreTake(xMutexModem, pdMS_TO_TICKS(5000))) {
            gprsUp = ensureGprs();
            xSemaphoreGive(xMutexModem);
        } else {
            Serial.println("[TEL] modem busy during GPRS check — skip");
            continue;
        }

        if (!gprsUp) {
            Serial.println("[TEL] no GPRS — skip");
            continue;
        }

        // Short mutex window for the actual HTTP POST only.
        if (!xSemaphoreTake(xMutexModem, pdMS_TO_TICKS(5000))) {
            Serial.println("[TEL] modem busy during POST — skip");
            continue;
        }

        {
            HttpClient http(gsmClient, SERVER_HOST, SERVER_PORT);
            int err = http.post(SERVER_PATH, "application/json", payload);
            if (err == HTTP_SUCCESS) {
                int statusCode = http.responseStatusCode();
                Serial.printf("[TEL] POST %d  (%d bytes)\n",
                              statusCode, payload.length());

                // Parse server response for OTA trigger.
                // Server may reply: { "ok": true, "ota_available": true }
                if (statusCode == 200) {
                    String body = http.responseBody();
                    JsonDocument resp;
                    if (deserializeJson(resp, body) == DeserializationError::Ok) {
                        if (resp["ota_available"] | false) {
                            xEventGroupSetBits(xEventAlerts, ALERT_OTA_AVAILABLE);
                            Serial.println("[TEL] server triggered OTA check");
                        }
                    }
                }
            } else {
                Serial.printf("[TEL] HTTP error %d\n", err);
            }
            http.stop();
        }

        xSemaphoreGive(xMutexModem);
    }
}
