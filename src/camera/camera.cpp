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

// Modbus CRC16 (polynomial 0xA001, init 0xFFFF, CRC sent low-byte first)
static uint16_t crc16Modbus(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// Build a Modbus RTU FC=0x03 (Read Holding Registers) request.
// buf must be at least 8 bytes. Returns frame length (always 8).
static size_t buildReadRegs(uint8_t* buf, uint8_t slave,
                             uint16_t regAddr, uint16_t regCount) {
    buf[0] = slave;
    buf[1] = 0x03;           // Function code: Read Holding Registers
    buf[2] = regAddr >> 8;
    buf[3] = regAddr & 0xFF;
    buf[4] = regCount >> 8;
    buf[5] = regCount & 0xFF;
    uint16_t crc = crc16Modbus(buf, 6);
    buf[6] = crc & 0xFF;     // CRC low byte first (Modbus RTU convention)
    buf[7] = crc >> 8;
    return 8;
}

static void sendFrame(const uint8_t* frame, size_t len) {
    rs485Tx();
    delayMicroseconds(100);   // DE line settling time
    Serial2.write(frame, len);
    Serial2.flush();
    delayMicroseconds(100);
    rs485Rx();
}

// Read exactly n bytes within timeoutMs. Returns true if all bytes arrived.
static bool readBytes(uint8_t* buf, size_t n, uint32_t timeoutMs) {
    size_t got = 0;
    uint32_t deadline = millis() + timeoutMs;
    while (got < n && millis() < deadline) {
        if (Serial2.available())
            buf[got++] = (uint8_t)Serial2.read();
        else
            vTaskDelay(pdMS_TO_TICKS(5));
    }
    return got == n;
}

// Validate an FC=0x03 response and extract register values.
// Expected frame: [slave][0x03][byteCount][dataHi][dataLo]...[crcLo][crcHi]
static bool parseResponse(const uint8_t* buf, size_t len, uint8_t slave,
                           uint16_t* regs, uint8_t regCount) {
    const size_t expected = 3u + (size_t)regCount * 2u + 2u;
    if (len < expected)            return false;
    if (buf[0] != slave)           return false;
    if (buf[1] != 0x03)            return false;
    if (buf[2] != (uint8_t)(regCount * 2)) return false;

    uint16_t crcCalc = crc16Modbus(buf, expected - 2);
    uint16_t crcRecv = (uint16_t)buf[expected - 2] | ((uint16_t)buf[expected - 1] << 8);
    if (crcCalc != crcRecv)        return false;

    for (uint8_t i = 0; i < regCount; i++)
        regs[i] = ((uint16_t)buf[3 + i * 2] << 8) | buf[4 + i * 2];
    return true;
}

void taskCamera(void* pvParameters) {
    pinMode(CAM_DE_RE_PIN, OUTPUT);
    rs485Rx();
    Serial2.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

    // Pre-build the request frame — it never changes at runtime
    uint8_t frame[8];
    size_t  frameLen = buildReadRegs(frame, CAM_SLAVE_ADDR,
                                     CAM_REG_FIRST, CAM_REG_COUNT);

    // Response: [slave][FC][byteCount] + 2*N data bytes + [crcLo][crcHi]
    uint8_t  response[3 + CAM_REG_COUNT * 2 + 2];
    uint16_t regs[CAM_REG_COUNT];

    for (;;) {
        // Wake on explicit trigger (e.g. from governor task) or periodic poll.
        xEventGroupWaitBits(xEventAlerts, ALERT_CAMERA_TRIG,
                            pdTRUE,   // clear bit on exit
                            pdFALSE,
                            pdMS_TO_TICKS(CAM_POLL_MS));

        // Discard any stale bytes from a previous incomplete exchange
        while (Serial2.available()) Serial2.read();

        sendFrame(frame, frameLen);

        const size_t respLen = sizeof(response);
        if (readBytes(response, respLen, 500) &&
            parseResponse(response, respLen, CAM_SLAVE_ADDR, regs, CAM_REG_COUNT)) {

            CameraData d;
            d.personCount = regs[0];
            d.confidence  = regs[1];  // TODO: verify register meaning for loaded model
            d.valid       = true;
            d.timestampMs = millis();

            if (xSemaphoreTake(xMutexCamera, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_latestCamera = d;
                xSemaphoreGive(xMutexCamera);
            }
            Serial.printf("[CAM] persons=%u conf=%u%%\n",
                          d.personCount, d.confidence);
        } else {
            Serial.println("[CAM] Modbus read timeout or CRC error");
        }
    }
}
