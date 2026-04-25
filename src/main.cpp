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

SemaphoreHandle_t  xMutexModem     = nullptr;
SemaphoreHandle_t  xMutexI2C       = nullptr;
SemaphoreHandle_t  xMutexGPS       = nullptr;
SemaphoreHandle_t  xMutexOccupancy = nullptr;
EventGroupHandle_t xEventAlerts    = nullptr;

// ── SIM7000G power-on sequence ───────────────────────────────────────────────
// 1. Enable SY8089 4.2 V supply (Sim.Pwr.En, active HIGH).
// 2. Pulse PWRKEY LOW for ≥1 s via Q5 NPN transistor:
//      GPIO HIGH → Q5 saturates → PWRKEY pulled LOW → module boots.
static void powerOnModem() {
    // Step 1 — enable dedicated SIM supply before asserting PWRKEY
    pinMode(SIM_PWR_EN_PIN, OUTPUT);
    digitalWrite(SIM_PWR_EN_PIN, HIGH);
    delay(200);   // allow SY8089 to ramp up and stabilise

    // Step 2 — assert PWRKEY through Q5 (GPIO HIGH = transistor on = PWRKEY LOW)
    pinMode(SIM_PWRKEY_PIN, OUTPUT);
    digitalWrite(SIM_PWRKEY_PIN, LOW);    // ensure deasserted first
    delay(50);
    digitalWrite(SIM_PWRKEY_PIN, HIGH);   // assert: Q5 on → PWRKEY LOW
    delay(1200);                           // SIM7000G requires ≥1 s
    digitalWrite(SIM_PWRKEY_PIN, LOW);    // deassert
    delay(3000);                           // wait for module to become ready
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

    xMutexModem     = xSemaphoreCreateMutex();
    xMutexI2C       = xSemaphoreCreateMutex();
    xMutexGPS       = xSemaphoreCreateMutex();
    xMutexOccupancy = xSemaphoreCreateMutex();
    xEventAlerts    = xEventGroupCreate();

    governorInit();   // relay + buzzer safe state FIRST, before anything else

    if (!initModem()) {
        Serial.println("[INIT] modem failed — governor + occupancy still active");
    }

    // Safety-critical tasks pinned to Core 1
    xTaskCreatePinnedToCore(taskGovernor,  "GOVERNOR",  3072, nullptr, 20, nullptr, 1);
    xTaskCreatePinnedToCore(taskGPS,       "GPS",       4096, nullptr, 15, nullptr, 1);

    // Support tasks on Core 0
    xTaskCreatePinnedToCore(taskCamera,    "CAMERA",    4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(taskOccupancy, "OCCUPANCY", 2048, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(taskTelemetry, "TELEMETRY", 8192, nullptr,  5, nullptr, 0);

    Serial.println("[INIT] all tasks started");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
