#pragma once

// Call once in setup() before any task is created — sets relay to safe state.
void governorInit();

void taskGovernor(void* pvParameters);
