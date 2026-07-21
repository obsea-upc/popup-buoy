#include "logging.h"
#include "conf.h"      // for SERIAL_DEBUG
#include <SD.h>
#include <RTClib.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern int currentState;
extern File LogFile;
extern const char *Log_filename;
extern RTC_DS3231 rtcExt;

void SerialPrintDebug(int message) {
  #ifdef SERIAL_DEBUG
    Serial.print(message);
  #endif
}

void SerialPrintDebugln(int message) {
  #ifdef SERIAL_DEBUG
    Serial.println(message);
  #endif
}

void SerialPrintDebug(String message) {
  #ifdef SERIAL_DEBUG
    Serial.print(message);
  #endif
}

void SerialPrintDebugln(String message) {
  #ifdef SERIAL_DEBUG
    Serial.println(message);
  #endif
}

bool writeLogFile(String message) {
  message = "State " + String(stateName(currentState)) + " - " + message;

  SerialPrintDebug("Writing in LogFile.txt ---");
  SerialPrintDebugln(message);

  // Open file and create if it doesn't exist
  LogFile = SD.open(Log_filename, FILE_APPEND);
  if (!LogFile) {
    LogFile.close();  // Closing the file
    return false;
  }

  DateTime timeRtcExt = rtcExt.now();  // To have the UTC time and not the local one
  if (!LogFile.print(String(timeRtcExt.timestamp(DateTime::TIMESTAMP_FULL)))) {
    return false;
  }

  if (!LogFile.print("----")) {
    return false;
  }

  if (!LogFile.println(message)) {
    return false;
  }

  LogFile.close();  // Closing the file
  return true;
}
