#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <Preferences.h>
#include <esp_ota_ops.h>
#include <mbedtls/sha256.h>
#include <ArduinoJson.h>
#include <TinyGsmClient.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "../shared/modem_shared.h"
#include "ota.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "0.0.0"
#endif

// ── NVS keys (namespace "ota") ────────────────────────────────────────────────
static const char* kNvsNs      = "ota";
static const char* kNvsLastVer = "last_ver";   // last version we attempted (avoid tight retry loops)

// ── Manifest ─────────────────────────────────────────────────────────────────
struct OtaManifest {
    char   version[32];
    char   url[128];     // URL path on OTA_SERVER_HOST — e.g. /api/v1/firmware/bunibox-1.0.1.bin
    char   sha256[65];   // 64 hex chars + null terminator
    size_t size;         // total binary size in bytes
};

// ── Helpers ───────────────────────────────────────────────────────────────────

static bool hexToBytes(const char* hex, uint8_t* out, size_t outLen) {
    if (strlen(hex) != outLen * 2) return false;
    for (size_t i = 0; i < outLen; i++) {
        auto h2n = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        int hi = h2n(hex[i * 2]);
        int lo = h2n(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

// Skip HTTP response headers; returns true when the blank separator line is found.
// Caller must provide a TinyGsmClient that has already received its status line.
static bool skipHttpHeaders(TinyGsmClient& client, uint32_t timeoutMs) {
    String prev = "_";   // non-empty sentinel so first real line doesn't match
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        if (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (line.length() == 0 && prev.length() == 0) return true;
            prev = line;
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    return false;
}

// Send a GET request with an optional Range header. Returns true if the HTTP
// status line contains the expected statusStr (e.g. "200" or "206").
static bool httpGet(TinyGsmClient& client, const char* host, uint16_t port,
                    const char* path, size_t rangeStart, size_t rangeEnd,
                    const char* expectedStatus) {
    if (!client.connect(host, port)) return false;

    client.print("GET "); client.print(path); client.println(" HTTP/1.0");
    client.print("Host: "); client.println(host);
    if (rangeEnd > rangeStart) {
        client.printf("Range: bytes=%u-%u\r\n",
                      (unsigned)rangeStart, (unsigned)rangeEnd);
    }
    client.println("Connection: close");
    client.println();

    // Read status line
    uint32_t t0 = millis();
    while (!client.available() && millis() - t0 < 8000) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    String statusLine = client.readStringUntil('\n');
    statusLine.trim();

    if (statusLine.indexOf(expectedStatus) < 0) {
        Serial.printf("[OTA] unexpected HTTP status: %s\n", statusLine.c_str());
        client.stop();
        return false;
    }
    return true;
}

// ── Manifest fetch ────────────────────────────────────────────────────────────
// Caller must hold xMutexModem.

static bool fetchManifest(TinyGsmClient& client, OtaManifest& m) {
    // Accept both 200 (standard) and 206 (unexpected for manifest, but safe)
    if (!httpGet(client, OTA_SERVER_HOST, OTA_SERVER_PORT,
                 OTA_MANIFEST_PATH, 0, 0, "200")) {
        return false;
    }

    if (!skipHttpHeaders(client, 8000)) {
        client.stop();
        Serial.println("[OTA] manifest header timeout");
        return false;
    }

    // Manifest JSON is small — read it all at once (≤ 512 bytes expected)
    String body;
    uint32_t t0 = millis();
    while (millis() - t0 < 10000) {
        while (client.available()) {
            body += (char)client.read();
        }
        if (!client.connected()) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    client.stop();

    if (body.isEmpty()) {
        Serial.println("[OTA] empty manifest body");
        return false;
    }

    JsonDocument doc;
    if (deserializeJson(doc, body) != DeserializationError::Ok) {
        Serial.println("[OTA] manifest JSON parse error");
        return false;
    }

    strlcpy(m.version, doc["version"] | "", sizeof(m.version));
    strlcpy(m.url,     doc["url"]     | "", sizeof(m.url));
    strlcpy(m.sha256,  doc["sha256"]  | "", sizeof(m.sha256));
    m.size = doc["size"] | 0;

    bool valid = (m.size > 0)
              && (strlen(m.url) > 0)
              && (strlen(m.version) > 0)
              && (strlen(m.sha256) == 64);
    if (!valid) {
        Serial.println("[OTA] manifest fields missing or invalid");
    }
    return valid;
}

// ── Download + flash ──────────────────────────────────────────────────────────
// Downloads the firmware in OTA_CHUNK_SIZE pieces.  The modem mutex is taken
// per-chunk so that GPS and telemetry can interleave between chunks.
// Returns true only when all bytes are written, SHA-256 verified, and the boot
// partition has been updated — after which it calls esp_restart() and never
// returns.

static bool downloadAndFlash(const OtaManifest& m) {
    const esp_partition_t* part = esp_ota_get_next_update_partition(nullptr);
    if (!part) {
        Serial.println("[OTA] no update partition");
        return false;
    }

    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(part, m.size, &handle) != ESP_OK) {
        Serial.println("[OTA] esp_ota_begin failed");
        return false;
    }

    uint8_t* buf = (uint8_t*)malloc(OTA_CHUNK_SIZE);
    if (!buf) {
        Serial.println("[OTA] out of heap for chunk buffer");
        esp_ota_abort(handle);
        return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);   // 0 = SHA-256 (not SHA-224)

    size_t  offset  = 0;
    bool    success = false;
    esp_err_t err   = ESP_OK;

    Serial.printf("[OTA] starting download: %s (%u bytes)\n",
                  m.url, (unsigned)m.size);

    while (offset < m.size) {
        size_t chunkEnd = offset + OTA_CHUNK_SIZE - 1;
        if (chunkEnd >= m.size) chunkEnd = m.size - 1;
        size_t chunkLen = chunkEnd - offset + 1;

        // ── Take modem mutex for this chunk ──────────────────────────────────
        if (!xSemaphoreTake(xMutexModem, pdMS_TO_TICKS(10000))) {
            Serial.println("[OTA] modem mutex timeout — aborting");
            break;
        }

        // Ensure GPRS is still up before each chunk
        if (!modem.isGprsConnected()) {
            Serial.println("[OTA] GPRS dropped — reconnecting");
            if (!modem.gprsConnect(CELL_APN, "", "")) {
                xSemaphoreGive(xMutexModem);
                Serial.println("[OTA] GPRS reconnect failed — aborting");
                break;
            }
        }

        // Try up to 3 times to fetch this chunk
        size_t received = 0;
        bool   chunkOk  = false;
        for (int attempt = 0; attempt < 3 && !chunkOk; attempt++) {
            TinyGsmClient client(modem, 1);   // socket 1 (telemetry uses 0)

            // 206 Partial Content is the correct response for a Range request;
            // also accept 200 in case the server ignores Range but returns all bytes.
            if (!httpGet(client, OTA_SERVER_HOST, OTA_SERVER_PORT,
                         m.url, offset, chunkEnd, "20")) {
                // "20" matches both "200" and "206"
                Serial.printf("[OTA] chunk connect/request failed (attempt %d)\n", attempt + 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            if (!skipHttpHeaders(client, 8000)) {
                client.stop();
                Serial.printf("[OTA] chunk header timeout (attempt %d)\n", attempt + 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }

            // Read exactly chunkLen bytes
            received = 0;
            uint32_t t0 = millis();
            while (received < chunkLen && millis() - t0 < 30000) {
                int avail = client.available();
                if (avail > 0) {
                    size_t want = min((size_t)avail, chunkLen - received);
                    int got = client.read(buf + received, want);
                    if (got > 0) received += (size_t)got;
                } else if (!client.connected()) {
                    break;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
            }
            client.stop();

            if (received == chunkLen) {
                chunkOk = true;
            } else {
                Serial.printf("[OTA] short read: got %u want %u (attempt %d)\n",
                              (unsigned)received, (unsigned)chunkLen, attempt + 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }

        xSemaphoreGive(xMutexModem);
        // ── Modem mutex released — GPS and telemetry can run ─────────────────

        if (!chunkOk) break;

        // Write chunk to OTA partition (no mutex needed — flash op is local)
        err = esp_ota_write(handle, buf, received);
        if (err != ESP_OK) {
            Serial.printf("[OTA] flash write error: %s\n", esp_err_to_name(err));
            break;
        }

        mbedtls_sha256_update(&sha, buf, received);
        offset += received;

        Serial.printf("[OTA] %u / %u bytes (%.0f%%)\n",
                      (unsigned)offset, (unsigned)m.size,
                      100.0f * offset / m.size);

        if (offset >= m.size) success = true;

        vTaskDelay(pdMS_TO_TICKS(50));   // yield — let other tasks breathe
    }

    free(buf);

    if (!success) {
        mbedtls_sha256_free(&sha);
        esp_ota_abort(handle);
        return false;
    }

    // ── SHA-256 verification ──────────────────────────────────────────────────
    uint8_t computed[32];
    mbedtls_sha256_finish(&sha, computed);
    mbedtls_sha256_free(&sha);

    uint8_t expected[32];
    if (!hexToBytes(m.sha256, expected, 32)) {
        Serial.println("[OTA] bad sha256 hex in manifest");
        esp_ota_abort(handle);
        return false;
    }

    if (memcmp(computed, expected, 32) != 0) {
        Serial.println("[OTA] SHA-256 MISMATCH — image rejected");
        esp_ota_abort(handle);
        return false;
    }
    Serial.println("[OTA] SHA-256 OK");

    // ── Commit ────────────────────────────────────────────────────────────────
    if (esp_ota_end(handle) != ESP_OK) {
        Serial.println("[OTA] esp_ota_end failed");
        return false;
    }

    if (esp_ota_set_boot_partition(part) != ESP_OK) {
        Serial.println("[OTA] set_boot_partition failed");
        return false;
    }

    Serial.println("[OTA] image committed — rebooting in 3 s");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
    return true;   // unreachable
}

// ── Public API ────────────────────────────────────────────────────────────────

void otaMarkBootValid() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (!running) return;   // not running from an OTA slot — direct flash, skip
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;
    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        Serial.println("[OTA] boot confirmed — rollback cancelled");
    }
}

void taskOTA(void* pvParameters) {
    // Give other tasks time to start up before the first version check.
    vTaskDelay(pdMS_TO_TICKS(20000));

    for (;;) {
        // Wait for a server-triggered notification OR the 6-hour polling interval.
        // ALERT_OTA_AVAILABLE is cleared (pdTRUE) on return so we don't re-enter
        // immediately on the next loop iteration.
        xEventGroupWaitBits(xEventAlerts,
                            ALERT_OTA_AVAILABLE,
                            pdTRUE,   // clear on exit
                            pdFALSE,  // wait for any bit (only one defined here)
                            pdMS_TO_TICKS(OTA_POLL_INTERVAL_MS));

        // Safety: don't update a moving vehicle.
        {
            GpsData gps = {};
            xSemaphoreTake(xMutexGPS, portMAX_DELAY);
            gps = g_latestGPS;
            xSemaphoreGive(xMutexGPS);

            if (gps.valid && gps.speedKmh > OTA_MAX_SPEED_KMH) {
                Serial.printf("[OTA] blocked — speed %.1f km/h\n", gps.speedKmh);
                continue;
            }
        }

        // ── Fetch manifest (take modem mutex for the short HTTP GET) ──────────
        if (!xSemaphoreTake(xMutexModem, pdMS_TO_TICKS(15000))) {
            Serial.println("[OTA] modem busy during manifest fetch — will retry");
            continue;
        }

        if (!modem.isGprsConnected()) {
            if (!modem.gprsConnect(CELL_APN, "", "")) {
                xSemaphoreGive(xMutexModem);
                Serial.println("[OTA] no GPRS — skipping check");
                continue;
            }
        }

        TinyGsmClient manifestClient(modem, 1);
        OtaManifest manifest = {};
        bool gotManifest = fetchManifest(manifestClient, manifest);
        xSemaphoreGive(xMutexModem);

        if (!gotManifest) continue;

        // ── Version comparison ────────────────────────────────────────────────
        if (strcmp(manifest.version, FIRMWARE_VERSION) == 0) {
            Serial.printf("[OTA] already on version %s\n", FIRMWARE_VERSION);
            continue;
        }

        // Avoid hammering a version that failed last boot.
        // Only skip if this is exactly the version we just attempted on the
        // previous boot (persisted in NVS), not just any old failure.
        {
            Preferences prefs;
            prefs.begin(kNvsNs, false);
            String lastAttempt = prefs.getString(kNvsLastVer, "");
            if (lastAttempt == manifest.version) {
                Serial.printf("[OTA] version %s previously attempted and failed — "
                              "waiting for next trigger\n", manifest.version);
                prefs.end();
                // Wait for an explicit server push before trying again
                xEventGroupWaitBits(xEventAlerts, ALERT_OTA_AVAILABLE,
                                    pdTRUE, pdFALSE, portMAX_DELAY);
                prefs.begin(kNvsNs, false);
            }
            prefs.putString(kNvsLastVer, manifest.version);
            prefs.end();
        }

        Serial.printf("[OTA] updating: %s → %s (%u bytes)\n",
                      FIRMWARE_VERSION, manifest.version, (unsigned)manifest.size);

        // downloadAndFlash holds the mutex per-chunk internally
        bool ok = downloadAndFlash(manifest);
        if (!ok) {
            Serial.println("[OTA] download failed — will retry on next trigger");
        }
        // On success, downloadAndFlash calls esp_restart() and never returns.
    }
}
