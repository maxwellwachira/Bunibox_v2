#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "occupancy.h"

// Read all 8 PCF8574 pins as a single byte via raw I2C (no extra library needed).
// PCF8574 pulls each pin HIGH internally; a sensor pulling a pin LOW = occupied.
static uint8_t pcfRead8() {
    Wire.requestFrom((uint8_t)PCF8574_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;  // 0xFF = all idle on read failure
}

void taskOccupancy(void* pvParameters) {
    xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    // Verify PCF8574 is present on the bus
    Wire.beginTransmission(PCF8574_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[OCC] PCF8574 not found — check wiring and address");
    }
    xSemaphoreGive(xMutexI2C);

    for (;;) {
        xSemaphoreTake(xMutexI2C, portMAX_DELAY);
        uint8_t raw = pcfRead8();
        xSemaphoreGive(xMutexI2C);

        // Invert: pin pulled LOW by sensor = occupied (bit = 1)
        uint8_t mask  = (~raw) & ((1 << SEAT_COUNT) - 1);
        uint8_t count = (uint8_t)__builtin_popcount(mask);

        xSemaphoreTake(xMutexOccupancy, portMAX_DELAY);
        g_latestOccupancy.seatMask      = mask;
        g_latestOccupancy.occupiedCount = count;
        g_latestOccupancy.timestampMs   = millis();
        xSemaphoreGive(xMutexOccupancy);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
