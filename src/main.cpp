#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "shared/data_types.h"
#include "shared/modem_shared.h"
#include "gps/gps.h"
#include "governor/governor.h"
#include "telemetry/telemetry.h"
#include "occupancy/occupancy.h"
#include "camera/camera.h"
#include "ota/ota.h"

// ── Single modem instance shared by gps.cpp and telemetry.cpp ───────────────
TinyGsm modem(Serial1);

// ── Shared state ─────────────────────────────────────────────────────────────
GpsData       g_latestGPS       = {};
OccupancyData g_latestOccupancy = {};
CameraData    g_latestCamera    = {};

SemaphoreHandle_t  xMutexModem     = nullptr;
SemaphoreHandle_t  xMutexI2C       = nullptr;
SemaphoreHandle_t  xMutexGPS       = nullptr;
SemaphoreHandle_t  xMutexOccupancy = nullptr;
SemaphoreHandle_t  xMutexCamera    = nullptr;
EventGroupHandle_t xEventAlerts    = nullptr;

// ── SIM7000G power-on sequence ───────────────────────────────────────────────
// 1. Enable SY8089 4.2 V supply (Sim.Pwr.En, active HIGH).
// 2. Pulse PWRKEY LOW for ≥1 s via Q5 NPN transistor:
//      GPIO HIGH → Q5 saturates → PWRKEY pulled LOW → module boots.
static void powerOnModem() {
    // Deassert NRESET (active LOW) — pin is 0Ω-direct to SIM7000G NRESET;
    // if left floating it can drift LOW and hold the module in permanent reset.
    pinMode(SIM_NRST_PIN, OUTPUT);
    digitalWrite(SIM_NRST_PIN, HIGH);

    // Wake module from sleep — DTR LOW = active/awake on SIM7000G.
    // If DTR is asserted (HIGH) the module ignores UART entirely.
    pinMode(SIM_DTR_PIN, OUTPUT);
    digitalWrite(SIM_DTR_PIN, LOW);
    delay(100);

    // Step 1 — enable dedicated SIM supply before asserting PWRKEY
    pinMode(SIM_PWR_EN_PIN, OUTPUT);
    digitalWrite(SIM_PWR_EN_PIN, HIGH);
    delay(500);   // allow SY8089 to ramp up fully (was 200 ms — too short)

    // Guard: if the module is already responsive (ESP32 reset without SIM
    // power-cycle), pulsing PWRKEY would TOGGLE IT OFF.  Two attempts cover
    // the case where the module is mid-URC (RDY / +CPIN) after a warm restart
    // and TinyGSM misses the first OK.
    for (int g = 0; g < 2; g++) {
        if (modem.testAT(3000)) {
            Serial.println("[INIT] modem already on — skipping PWRKEY");
            return;
        }
        delay(500);
    }

    // Step 2 — assert PWRKEY through Q5 (GPIO HIGH = transistor on = PWRKEY LOW)
    Serial.println("[INIT] pulsing PWRKEY...");
    pinMode(SIM_PWRKEY_PIN, OUTPUT);
    digitalWrite(SIM_PWRKEY_PIN, LOW);    // ensure deasserted first
    delay(50);
    digitalWrite(SIM_PWRKEY_PIN, HIGH);   // assert: Q5 on → PWRKEY LOW
    delay(1500);                           // SIM7000G requires ≥1 s; 1.5 s is safe margin
    digitalWrite(SIM_PWRKEY_PIN, LOW);    // deassert
    delay(7000);                           // allow up to 7 s for full boot + RDY output
}

static bool initModem() {
    Serial1.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

    powerOnModem();

    // Retry testAT — never pulse PWRKEY again (second pulse toggles it OFF).
    bool modemOk = false;
    for (int i = 1; i <= 3; i++) {
        Serial.printf("[INIT] modem AT (%d/3) ", i);
        if (modem.testAT(10000)) { Serial.println("OK"); modemOk = true; break; }
        Serial.println("FAIL");
    }

    // Last resort: hardware reset via NRST if PWRKEY boot failed.
    if (!modemOk) {
        Serial.println("[INIT] PWRKEY boot failed — trying NRST hard reset");
        digitalWrite(SIM_NRST_PIN, LOW);
        delay(200);                        // hold NRESET LOW ≥105 ms (datasheet)
        digitalWrite(SIM_NRST_PIN, HIGH);
        delay(3000);                       // boot after hard reset is faster (~3 s)
        if (!modem.testAT(5000)) {
            Serial.println("[INIT] NRST reset failed — modem unresponsive");
            return false;
        }
        Serial.println("[INIT] modem recovered via NRST");
    }

    // Full modem init: sends ATE0 (echo off) + checks AT+CPIN? (SIM PIN status).
    // Must come before any command whose response TinyGSM needs to parse — echo
    // on corrupts response parsing and makes registration checks silently fail.
    Serial.print("[INIT] modem init ");
    if (!modem.init()) {
        Serial.println("FAIL");
        return false;
    }
    Serial.println("OK");

    // Check SIM status explicitly so we know whether PIN is needed.
    String simStatus;
    switch (modem.getSimStatus()) {
        case SIM_READY:     simStatus = "READY";           break;
        case SIM_LOCKED:    simStatus = "PIN LOCKED";      break;
        case SIM_ANTITHEFT_LOCKED: simStatus = "PUK LOCKED"; break;
        default:            simStatus = "NOT INSERTED / UNKNOWN"; break;
    }
    Serial.printf("[INIT] SIM status: %s\n", simStatus.c_str());
    if (modem.getSimStatus() != SIM_READY) {
        Serial.println("[INIT] SIM not ready — aborting");
        return false;
    }

    Serial.printf("[INIT] IMEI=%s\n",     modem.getIMEI().c_str());
    Serial.printf("[INIT] ICCID=%s\n",    modem.getSimCCID().c_str());
    Serial.printf("[INIT] modem fw=%s\n", modem.getModemInfo().c_str());

    modem.enableGPS();   // AT+CGNSPWR=1

    auto signalLabel = [](int rssi) -> String {
        if (rssi == 99) return "no signal";
        if (rssi == 0)  return "≤-113 dBm";
        if (rssi == 31) return "≥-51 dBm";
        return String(-113 + 2 * rssi) + " dBm";
    };

    // Helper: attempt GPRS attach + PDP activation and log the result.
    // gprsConnect() sends AT+CGATT=1 + AT+CGDCONT + AT+CGACT internally,
    // so it handles the full attach sequence — waitForNetwork only checks
    // registration status and is not needed before gprsConnect.
    auto tryGprs = [&](const char* label) -> bool {
        int rssi = modem.getSignalQuality();
        Serial.printf("[INIT] GPRS connecting on %s  signal=%s  APN=%s...\n",
                      label, signalLabel(rssi).c_str(), CELL_APN);
        if (!modem.gprsConnect(CELL_APN, "", "")) {
            Serial.printf("[INIT] GPRS FAIL on %s  signal=%s"
                          "  (telemetry task will retry)\n",
                          label, signalLabel(modem.getSignalQuality()).c_str());
            return false;
        }
        Serial.printf("[INIT] GPRS OK  operator=%s  signal=%s  IP=%s\n",
                      modem.getOperator().c_str(),
                      signalLabel(modem.getSignalQuality()).c_str(),
                      modem.getLocalIP().c_str());
        return true;
    };

    // Use auto (GSM+LTE) directly — no LTE antenna fitted, so Cat-M1 always
    // fails and the probe just wastes boot time.  Switch to LTE-only in config
    // once an LTE antenna is installed.
    // CNMP=2 = auto (GSM + LTE), CMNB=3 = Cat-M1 + NB-IoT (GSM via CNMP).
    Serial.println("[INIT] network mode: auto (GSM+LTE)");
    modem.setNetworkMode(2);
    modem.setPreferredMode(3);
    delay(1000);   // allow radio to start scanning before connect attempt
    tryGprs("auto/GSM");

    return true;
}

// ── Arduino entry points ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== BuniBox booting ===");

    // gprsConnect() can block for up to 60 s without yielding, starving IDLE0
    // and triggering the default 5-second task watchdog.  Extend to 90 s so
    // legitimate reconnect attempts don't crash the device.
    // IDF 4.4 has no reconfigure(); deinit + reinit + re-subscribe idle tasks.
    esp_task_wdt_deinit();
    esp_task_wdt_init(120, true);  // 120 s: gprsConnect() can block ~85 s on AT+CIICR
    for (int cpu = 0; cpu < portNUM_PROCESSORS; cpu++) {
        esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(cpu));
    }

    xMutexModem     = xSemaphoreCreateMutex();
    xMutexI2C       = xSemaphoreCreateMutex();
    xMutexGPS       = xSemaphoreCreateMutex();
    xMutexOccupancy = xSemaphoreCreateMutex();
    xMutexCamera    = xSemaphoreCreateMutex();
    xEventAlerts    = xEventGroupCreate();

#ifdef SEAT_TEST_ONLY
    Serial.println("[INIT] *** SEAT_TEST_ONLY mode — modem/GPS/telemetry/camera disabled ***");
    xTaskCreatePinnedToCore(taskOccupancy, "OCCUPANCY", 2048, nullptr, 10, nullptr, 0);
#else
    governorInit();   // relay + buzzer safe state FIRST, before anything else

    if (!initModem()) {
        Serial.println("[INIT] modem failed — governor + occupancy still active");
    }

    // Mark the running image valid if this boot follows an OTA update.
    // Must be called before tasks start so the check happens even if a task
    // panics early.  otaMarkBootValid() is a no-op on normal (non-OTA) boots.
    otaMarkBootValid();

    // Safety-critical tasks pinned to Core 1
    xTaskCreatePinnedToCore(taskGovernor,  "GOVERNOR",  3072, nullptr, 20, nullptr, 1);
    xTaskCreatePinnedToCore(taskGPS,       "GPS",       4096, nullptr, 15, nullptr, 1);

    // Support tasks on Core 0
    xTaskCreatePinnedToCore(taskCamera,    "CAMERA",    4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(taskOccupancy, "OCCUPANCY", 2048, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(taskTelemetry, "TELEMETRY", 8192, nullptr,  5, nullptr, 0);
    // OTA: lower priority than telemetry so normal operation is never starved.
    // Stack is generous because JSON parsing + TLS handshake live on this stack.
    xTaskCreatePinnedToCore(taskOTA,       "OTA",      10240, nullptr,  3, nullptr, 0);
#endif

    Serial.println("[INIT] all tasks started");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
