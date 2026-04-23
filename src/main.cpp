#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include "config.h"
#include "shared/data_types.h"
#include "shared/modem_shared.h"
#include "gps/gps.h"
#include "governor/governor.h"
#include "telemetry/telemetry.h"
#include "occupancy/occupancy.h"
#include "camera/camera.h"

// ── Single modem instance shared by gps.cpp and telemetry.cpp ───────────────
TinyGsm modem(Serial1);

// ── Shared state ─────────────────────────────────────────────────────────────
GpsData       g_latestGPS       = {};
OccupancyData g_latestOccupancy = {};

SemaphoreHandle_t  xMutexModem    = nullptr;
SemaphoreHandle_t  xMutexI2C      = nullptr;
SemaphoreHandle_t  xMutexGPS      = nullptr;
SemaphoreHandle_t  xMutexOccupancy = nullptr;
EventGroupHandle_t xEventAlerts   = nullptr;

// ── Modem power-on ───────────────────────────────────────────────────────────
static void powerOnModem() {
    pinMode(SIM_PWRKEY_PIN, OUTPUT);
    digitalWrite(SIM_PWRKEY_PIN, HIGH);
    delay(100);
    digitalWrite(SIM_PWRKEY_PIN, LOW);   // pull LOW ≥1 s to trigger boot
    delay(1200);
    digitalWrite(SIM_PWRKEY_PIN, HIGH);
    delay(3000);                          // wait for SIM7000G to become ready
}

static bool initModem() {
    Serial1.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
    powerOnModem();

    Serial.print("[INIT] modem AT ");
    if (!modem.testAT(10000)) {
        Serial.println("FAIL");
        return false;
    }
    Serial.println("OK");

    modem.enableGPS();   // AT+CGNSPWR=1

    Serial.print("[INIT] GPRS ");
    if (!modem.gprsConnect(CELL_APN, "", "")) {
        Serial.println("FAIL (telemetry task will retry)");
    } else {
        Serial.println("OK");
    }

    return true;
}

// ── Arduino entry points ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== BuniBox booting ===");

    // Create all synchronisation primitives before any task starts
    xMutexModem     = xSemaphoreCreateMutex();
    xMutexI2C       = xSemaphoreCreateMutex();
    xMutexGPS       = xSemaphoreCreateMutex();
    xMutexOccupancy = xSemaphoreCreateMutex();
    xEventAlerts    = xEventGroupCreate();

    governorInit();   // relay safe state FIRST — before modem or network

    if (!initModem()) {
        Serial.println("[INIT] modem failed — governor + occupancy still active");
    }

    // Safety-critical tasks pinned to Core 1 (away from any background work on Core 0)
    xTaskCreatePinnedToCore(taskGovernor,  "GOVERNOR",  3072, nullptr, 20, nullptr, 1);
    xTaskCreatePinnedToCore(taskGPS,       "GPS",       4096, nullptr, 15, nullptr, 1);

    // Support tasks on Core 0
    xTaskCreatePinnedToCore(taskCamera,    "CAMERA",    4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(taskOccupancy, "OCCUPANCY", 2048, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(taskTelemetry, "TELEMETRY", 8192, nullptr,  5, nullptr, 0);

    Serial.println("[INIT] all tasks started");
}

void loop() {
    vTaskDelay(portMAX_DELAY);   // hand control to FreeRTOS scheduler
}
