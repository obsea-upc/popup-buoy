#pragma once
#include <Arduino.h>

// Serial debug helpers. They compile to no-ops unless SERIAL_DEBUG is defined (see conf.h).
void SerialPrintDebug(int message);
void SerialPrintDebugln(int message);
void SerialPrintDebug(String message);
void SerialPrintDebugln(String message);

// Append a timestamped, state-prefixed line to the SD log file (and echo it over serial debug).
bool writeLogFile(String message);

// SD log file path (defined in logging.cpp).
extern const char *Log_filename;
