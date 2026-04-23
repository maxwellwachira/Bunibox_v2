#pragma once

// ── SIM7000G (UART1) ────────────────────────────────────────────────────────
#define SIM_RX_PIN          18
#define SIM_TX_PIN          17
#define SIM_PWRKEY_PIN       4    // Pull LOW ≥1s to power on; adjust if board inverts via transistor
#define SIM_BAUD        115200

// ── RS485 Camera (UART2) ────────────────────────────────────────────────────
#define CAM_RX_PIN          15
#define CAM_TX_PIN          16
#define CAM_DE_RE_PIN        7    // HIGH = transmit mode, LOW = receive mode
#define CAM_BAUD        115200

// ── I2C — PCF8574 seat sensors ──────────────────────────────────────────────
#define I2C_SDA_PIN          8
#define I2C_SCL_PIN          9
#define PCF8574_ADDR      0x20    // A0=A1=A2 tied to GND; change per board

// ── Relay (fuel pump cutoff) ─────────────────────────────────────────────────
// Wiring: relay NC contact in series with fuel pump.
// Energising relay (RELAY_ACTIVE) opens NC → cuts fuel.
// On power loss the NC contact closes → fuel flows (fail-safe).
#define RELAY_PIN            5
#define RELAY_ACTIVE       LOW    // LOW energises relay; change to HIGH if your driver inverts

// ── Speed governor ───────────────────────────────────────────────────────────
#define SPEED_LIMIT_KMH   80.0f
#define SPEED_RESTORE_KMH 75.0f  // hysteresis: restore fuel only when speed drops below this
#define GPS_POLL_MS        500

// ── Cellular / telemetry ─────────────────────────────────────────────────────
#define CELL_APN        "your-apn-here"
#define SERVER_HOST     "api.yourdomain.com"
#define SERVER_PORT      80
#define SERVER_PATH     "/api/v1/telemetry"
#define TELEMETRY_INTERVAL_MS  10000UL

// ── Seat count ───────────────────────────────────────────────────────────────
#define SEAT_COUNT           8    // one PCF8574 = 8 seats; add more chips for more seats
