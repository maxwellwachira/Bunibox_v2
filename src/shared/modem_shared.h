#pragma once
#include <TinyGsmClient.h>

// TinyGsm modem instance — defined once in main.cpp, used by gps.cpp and telemetry.cpp.
// Both translation units must take xMutexModem before calling any modem method.
extern TinyGsm modem;
