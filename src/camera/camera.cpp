#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "../config.h"
#include "../shared/data_types.h"
#include "camera.h"

static inline void rs485Tx() { digitalWrite(CAM_DE_RE_PIN, HIGH); }
static inline void rs485Rx() { digitalWrite(CAM_DE_RE_PIN, LOW);  }

static uint16_t crc16Modbus(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static size_t buildReadRegs(uint8_t* buf, uint8_t slave,
                             uint16_t regAddr, uint16_t regCount) {
    buf[0] = slave;
    buf[1] = 0x03;
    buf[2] = regAddr >> 8;
    buf[3] = regAddr & 0xFF;
    buf[4] = regCount >> 8;
    buf[5] = regCount & 0xFF;
    uint16_t crc = crc16Modbus(buf, 6);
    buf[6] = crc & 0xFF;
    buf[7] = crc >> 8;
    return 8;
}

static void sendFrame(const uint8_t* frame, size_t len) {
    rs485Tx();
    delayMicroseconds(100);
    Serial2.write(frame, len);
    Serial2.flush();
    delayMicroseconds(100);
    rs485Rx();
}

// Returns number of bytes actually read (may be < n on timeout).
static size_t readBytes(uint8_t* buf, size_t n, uint32_t timeoutMs) {
    size_t got = 0;
    uint32_t deadline = millis() + timeoutMs;
    while (got < n && millis() < deadline) {
        if (Serial2.available())
            buf[got++] = (uint8_t)Serial2.read();
        else
            vTaskDelay(pdMS_TO_TICKS(5));
    }
    return got;
}

static void logHex(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i++) Serial.printf(" %02X", buf[i]);
    Serial.println();
}

// Returns true on valid FC=0x03 response; fills regs[].
static bool parseResponse(const uint8_t* buf, size_t len, uint8_t slave,
                           uint16_t* regs, uint8_t regCount) {
    const size_t expected = 3u + (size_t)regCount * 2u + 2u;
    if (len < expected)                          return false;
    if (buf[0] != slave)                         return false;
    if (buf[1] != 0x03)                          return false;
    if (buf[2] != (uint8_t)(regCount * 2))       return false;
    uint16_t crcCalc = crc16Modbus(buf, expected - 2);
    uint16_t crcRecv = (uint16_t)buf[expected-2] | ((uint16_t)buf[expected-1] << 8);
    if (crcCalc != crcRecv)                      return false;
    for (uint8_t i = 0; i < regCount; i++)
        regs[i] = ((uint16_t)buf[3 + i*2] << 8) | buf[4 + i*2];
    return true;
}

void taskCamera(void* pvParameters) {
    pinMode(CAM_DE_RE_PIN, OUTPUT);
    rs485Rx();
    Serial2.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

    uint8_t frame[8];
    size_t  frameLen = buildReadRegs(frame, CAM_SLAVE_ADDR,
                                     CAM_REG_FIRST, CAM_REG_COUNT);

    // Camera needs a few seconds to boot and load the AI model before it
    // will answer Modbus queries. 6 s covers the longest observed startup time.
    Serial.printf("[CAM] waiting for camera boot (baud=%u slave=0x%02X)...\n",
                  CAM_BAUD, CAM_SLAVE_ADDR);
    vTaskDelay(pdMS_TO_TICKS(6000));

    // Verify DE/RE pin is actually toggling — if it stays LOW the transmitter
    // is always disabled and the camera never receives our request.
    rs485Tx();
    bool deHigh = digitalRead(CAM_DE_RE_PIN) == HIGH;
    rs485Rx();
    bool deLow  = digitalRead(CAM_DE_RE_PIN) == LOW;
    Serial.printf("[CAM] DE/RE pin check: TX=%s RX=%s%s\n",
                  deHigh ? "HIGH OK" : "HIGH FAIL",
                  deLow  ? "LOW OK"  : "LOW FAIL",
                  (deHigh && deLow) ? "" : " ← GPIO46 not toggling, check pin config");

    Serial.print("[CAM] request frame (hex):");
    logHex(frame, frameLen);

    // Normal FC=0x03 reply length; exception reply is 5 bytes
    const size_t respLen = 3 + CAM_REG_COUNT * 2 + 2;
    uint8_t  response[respLen + 4];   // +4 headroom for unexpected extra bytes
    uint16_t regs[CAM_REG_COUNT];

    for (;;) {
        xEventGroupWaitBits(xEventAlerts, ALERT_CAMERA_TRIG,
                            pdTRUE, pdFALSE,
                            pdMS_TO_TICKS(CAM_POLL_MS));

        while (Serial2.available()) Serial2.read();   // flush stale bytes

        sendFrame(frame, frameLen);

        size_t got = readBytes(response, respLen, 2000);

        if (got == 0) {
            // Nothing arrived at all — most likely a baud rate or wiring issue
            Serial.printf("[CAM] timeout — 0 bytes (check baud=%u, slave=0x%02X, wiring)\n",
                          CAM_BAUD, CAM_SLAVE_ADDR);

        } else if (got >= 5 && response[1] == (0x03 | 0x80)) {
            // Camera returned a Modbus exception response
            // Exception codes: 0x01=illegal function, 0x02=illegal address, 0x03=illegal value
            uint16_t excCrc = crc16Modbus(response, 3);
            bool crcOk = (response[3] == (excCrc & 0xFF)) && (response[4] == (excCrc >> 8));
            Serial.printf("[CAM] Modbus exception 0x%02X%s — register 0x%04X may be wrong\n",
                          response[2], crcOk ? "" : " (bad CRC)", CAM_REG_FIRST);

        } else if (got < respLen) {
            // Partial response — probably baud rate mismatch causing framing errors
            Serial.printf("[CAM] short response (%u/%u bytes) — raw:", got, respLen);
            logHex(response, got);

        } else if (!parseResponse(response, respLen, CAM_SLAVE_ADDR, regs, CAM_REG_COUNT)) {
            // Full-length response but CRC or framing wrong — baud rate or slave addr mismatch
            Serial.print("[CAM] CRC/framing error — raw:");
            logHex(response, respLen);

        } else {
            // Success
            CameraData d;
            d.personCount = regs[0];
            d.confidence  = regs[1];
            d.valid       = true;
            d.timestampMs = millis();

            if (xSemaphoreTake(xMutexCamera, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_latestCamera = d;
                xSemaphoreGive(xMutexCamera);
            }
            Serial.printf("[CAM] persons=%u conf=%u%%\n",
                          d.personCount, d.confidence);
        }
    }
}
