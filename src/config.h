#pragma once

// ── Test mode ─────────────────────────────────────────────────────────────────
// Uncomment to boot with ONLY the occupancy task running (no modem, no governor,
// no GPS, no telemetry, no camera).  Useful for bench-testing seat sensors.
//  #define SEAT_TEST_ONLY

// ── Bus seat layout ───────────────────────────────────────────────────────────
// Uncomment ONE line to match the physical wiring harness on the vehicle.
//   BUS_2ROW: 2× PCF8575 at 0x20 & 0x21 — up to 26 passenger + driver + co-pilot
//   BUS_3ROW: 3× PCF8575 at 0x22, 0x23, 0x24 — up to 39 passenger seats
// #define BUS_2ROW
#define BUS_3ROW

#ifdef BUS_2ROW
  #define OCC_CHIP_COUNT   2
  static const uint8_t kOccChipAddrs[OCC_CHIP_COUNT] = { 0x20, 0x21 };
#else
  #define OCC_CHIP_COUNT   3
  static const uint8_t kOccChipAddrs[OCC_CHIP_COUNT] = { 0x22, 0x23, 0x24 };
#endif

// ── SIM7000G  UART1 ───────────────────────────────────────────────────────────
// The module is powered by a dedicated SY8089 4.2 V buck (controlled by PWR_EN).
// PWRKEY goes through Q5 NPN transistor: GPIO HIGH → Q5 on → PWRKEY pulled LOW.
// NRST connects directly via R18 (0 Ω): GPIO LOW → SIM7000G hard reset.
#define SIM_TX_PIN        2   // GPIO2  → GSM.Txd → SIM7000G RXD (pin 10)
#define SIM_RX_PIN        4   // GPIO4  → GSM.Rxd → SIM7000G TXD (pin 9)
#define SIM_DTR_PIN       1   // GPIO1  → Sim.Dtr → SIM7000G DTR (pin 3)
#define SIM_PWR_EN_PIN    7   // GPIO7  → Sim.Pwr.En → SY8089 EN
#define SIM_PWRKEY_PIN    6   // GPIO6  → ~{Sim.PWRKEY} → Q5 base
#define SIM_NRST_PIN      5   // GPIO5  → ~{Sim.NRST} → R18 → NRESET (active LOW)
#define SIM_BAUD      115200

// ── RS485 AI Camera  UART2 ───────────────────────────────────────────────────
// Seeed RS485 Vision AI Camera (S-VA-01B). Modbus RTU slave over TP8485E-SR.
// DE and RE are tied together on CAM_DE_RE_PIN: HIGH=transmit, LOW=receive.
#define CAM_TX_PIN       17   // GPIO17 → RS485.TxD → TP8485E-SR D
#define CAM_RX_PIN       18   // GPIO18 → RS485.RxD → TP8485E-SR A/Y
#define CAM_DE_RE_PIN    46   // GPIO46 → RS485.EN  → TP8485E-SR DE/RE
#define CAM_INT_PIN      45   // GPIO45 → RS485.Int (camera interrupt to ESP32)  // TODO: verify
// TODO: verify baud rate — check SenseCraft App → Device → Protocol
// Try 115200 first (common Seeed default); fall back to 9600 if no response.
#define CAM_BAUD      115200

// Modbus RTU master settings
#define CAM_SLAVE_ADDR  0x01    // default slave address (configurable via SenseCraft App)
#define CAM_POLL_MS     1000UL  // normal polling interval when no trigger fires

// Holding register start address and count.
// TODO: confirm exact addresses from SenseCraft App → Device → Protocol tab or datasheet.
// With people-count model loaded, 0x0004 is expected to hold person count.
#define CAM_REG_FIRST   0x0004  // first register to read
#define CAM_REG_COUNT   2       // number of registers: [0]=person count, [1]=confidence

// ── I²C  ──────────────────────────────────────────────────────────────────────
// Shared bus: PCF8575 seat-occupancy chips + PCF8574AT display expander (U7).
// Pull-ups R149/R150 (4.7 kΩ each) are on the main controller PCB.
#define I2C_SDA_PIN       8   // GPIO8
#define I2C_SCL_PIN       9   // GPIO9

// ── Speed governor relay ──────────────────────────────────────────────────────
// Path: GPIO → R55(470Ω) → PC817 LED → phototransistor → R56 → Q8 NPN → G5Q-1A coil.
// Relay NC contact is in series with the fuel pump.
//   Relay energised (GPIO HIGH) → NC opens → fuel cut.
//   Relay deenergised / power lost → NC closes → fuel flows  (fail-safe).
#define RELAY_PIN        40   // GPIO40 → Speed.Ctrl
#define RELAY_ACTIVE   HIGH   // HIGH energises relay (via PC817 + Q8); cut = fuel off

// ── Buzzer ────────────────────────────────────────────────────────────────────
// Path: GPIO → R58(470Ω) → Q9 NPN base → Q9 collector → BZ1 buzzer → +5V.
// HIGH = buzzer on.
#define BUZZER_PIN       21   // GPIO21 → Buzzer.Fire

// ── Other peripheral GPIOs ───────────────────────────────────────────────────
#define DOOR_REED_1_PIN  35   // GPIO35 → Door.reed.data.1
#define DOOR_REED_2_PIN  36   // GPIO36 → Door.reed.data.2
#define TRIG_PIN         37   // GPIO37 → Trig (PIR / trigger detect)
#define RFID_RST_PIN     38   // GPIO38 → RFID.RST
#define LED_DEBUG_PIN    39   // GPIO39 → led.debug
#define IR_DATA_1_PIN    41   // GPIO41 → Infrared.Data.1
#define IR_DATA_2_PIN    42   // GPIO42 → Infrared.Data.2
#define DISP_RES_PIN     47   // GPIO47 → Disp.RES (PCF8574AT P0)
#define DISP_DC_PIN      48   // GPIO48 → Disp.DC  (PCF8574AT P1)

// ── SPI bus (RFID + main-board SD card, shared) ───────────────────────────────
#define SPI_MOSI_PIN     11   // TODO: verify from schematic
#define SPI_MISO_PIN     13   // TODO: verify from schematic
#define SPI_SCK_PIN      12   // TODO: verify from schematic
#define SPI_NSS_RFID     10   // RFID chip select  TODO: verify

// ── Speed limit ───────────────────────────────────────────────────────────────
#define SPEED_LIMIT_KMH   80.0f
#define SPEED_RESTORE_KMH 75.0f  // hysteresis: restore only when speed drops below this
#define GPS_POLL_MS        500

// ── Cellular / telemetry ─────────────────────────────────────────────────────
#define DEVICE_ID             "BB-47392"
#define CELL_APN              "safaricom"   // TODO: confirm APN with SIM carrier
#define SERVER_HOST           "172.104.159.240"
#define SERVER_PORT            3001  // bypass Safaricom port-80 transparent proxy
#define SERVER_PATH           "/api/v1/telemetry"
#define TELEMETRY_INTERVAL_MS  10000UL

// ── OTA firmware update ───────────────────────────────────────────────────────
// The OTA server can be the same host as the telemetry server.
// MANIFEST_PATH returns JSON: { version, url, sha256, size }
// FIRMWARE_PATH_PREFIX is the URL path to the raw .bin (same host/port).
#define OTA_SERVER_HOST       SERVER_HOST
#define OTA_SERVER_PORT       SERVER_PORT
#define OTA_MANIFEST_PATH     "/api/v1/firmware/latest"

// 8 KB per HTTP Range request — balances GPRS reliability vs. connection overhead.
// Each chunk holds the modem mutex for ~3–8 s; GPS/telemetry skip those cycles.
#define OTA_CHUNK_SIZE        (8 * 1024)

// Periodic version-check interval (6 hours). Telemetry response can also trigger
// an immediate check via the ALERT_OTA_AVAILABLE event bit.
#define OTA_POLL_INTERVAL_MS  (6UL * 60UL * 60UL * 1000UL)

// OTA is blocked while the vehicle is moving faster than this speed.
#define OTA_MAX_SPEED_KMH     5.0f
