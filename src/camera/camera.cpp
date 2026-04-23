#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "camera.h"

// RS485 half-duplex direction control
static inline void rs485Tx() { digitalWrite(CAM_DE_RE_PIN, HIGH); }
static inline void rs485Rx() { digitalWrite(CAM_DE_RE_PIN, LOW);  }

// TODO: replace with actual vendor command frame for your RS485 camera model
static const uint8_t CMD_CAPTURE[] = { 0x56, 0x00, 0x36, 0x01, 0x00 };

static void sendCommand(const uint8_t* cmd, size_t len) {
    rs485Tx();
    delayMicroseconds(100);    // DE settling time
    Serial2.write(cmd, len);
    Serial2.flush();
    delayMicroseconds(100);
    rs485Rx();
}

static bool waitAck(uint32_t timeoutMs) {
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (Serial2.available()) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

void taskCamera(void* pvParameters) {
    pinMode(CAM_DE_RE_PIN, OUTPUT);
    rs485Rx();
    Serial2.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

    for (;;) {
        // Block indefinitely until governor (or any other source) sets the trigger bit.
        // pdTRUE clears the bit automatically on exit so each trigger fires once.
        xEventGroupWaitBits(xEventAlerts, ALERT_CAMERA_TRIG,
                            pdTRUE,    // clear on exit
                            pdFALSE,
                            portMAX_DELAY);

        Serial.println("[CAM] trigger — sending capture command");
        sendCommand(CMD_CAPTURE, sizeof(CMD_CAPTURE));

        if (waitAck(3000)) {
            // TODO: read the full image frame per your camera's protocol.
            // Options: store to SPIFFS/SD, or hand a pointer to taskTelemetry
            // via a queue for immediate upload.
            Serial.println("[CAM] capture ack OK");
        } else {
            Serial.println("[CAM] capture timeout — no ack");
        }
    }
}
