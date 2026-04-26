#pragma once

// Call once from main.cpp after the first telemetry POST succeeds.
// Marks the running partition as valid, preventing automatic rollback to the
// previous firmware if this is a freshly-applied OTA image.
// Safe to call on every boot — it is a no-op if no rollback is pending.
void otaMarkBootValid();

// FreeRTOS task — create from main.cpp on Core 0 at priority 3.
// Waits for ALERT_OTA_AVAILABLE (set by telemetry task) or a 6-hour timeout,
// then downloads, verifies, and applies a firmware update over GPRS.
void taskOTA(void* pvParameters);
