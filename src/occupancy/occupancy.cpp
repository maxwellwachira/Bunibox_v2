#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "occupancy.h"

// Read all 16 pins of one PCF8575 as a uint16 via raw I2C (no external library).
// PCF8575 pulls each pin HIGH internally; a sensor grounding a pin = occupied.
// Returns raw data (HIGH=empty, LOW=occupied).  Returns 0xFFFF on bus error.
static uint16_t pcf8575Read(uint8_t addr) {
    Wire.requestFrom(addr, (uint8_t)2);
    if (Wire.available() < 2) return 0xFFFF;
    uint8_t lo = Wire.read();   // P00–P07
    uint8_t hi = Wire.read();   // P10–P17
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

// Probe the I2C bus for all configured chip addresses and log which are found.
static void detectChips() {
    Serial.printf("[OCC] scanning %d chip(s): ", OCC_CHIP_COUNT);
    for (int i = 0; i < OCC_CHIP_COUNT; i++) {
        Wire.beginTransmission(kOccChipAddrs[i]);
        uint8_t err = Wire.endTransmission();
        Serial.printf("0x%02X=%s ", kOccChipAddrs[i], err == 0 ? "OK" : "MISS");
    }
    Serial.println();
}

void taskOccupancy(void* pvParameters) {
    xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    detectChips();
    xSemaphoreGive(xMutexI2C);

    for (;;) {
        OccupancyData fresh = {};
        fresh.chipCount    = OCC_CHIP_COUNT;
        fresh.timestampMs  = millis();

        xSemaphoreTake(xMutexI2C, portMAX_DELAY);
        for (int i = 0; i < OCC_CHIP_COUNT; i++) {
            uint16_t raw = pcf8575Read(kOccChipAddrs[i]);
            // Invert: bit=1 means sensor pulled pin LOW → seat occupied.
            // On read error (0xFFFF) the inversion gives 0x0000 — all empty — safe default.
            fresh.chipData[i]   = ~raw;
            fresh.occupiedCount += (uint8_t)__builtin_popcount(fresh.chipData[i]);
        }
        xSemaphoreGive(xMutexI2C);

        xSemaphoreTake(xMutexOccupancy, portMAX_DELAY);
        g_latestOccupancy = fresh;
        xSemaphoreGive(xMutexOccupancy);

        Serial.printf("[OCC] %d seat(s) occupied\n", fresh.occupiedCount);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
