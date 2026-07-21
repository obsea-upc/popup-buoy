#include "eeprom_store.h"
#include "logging.h"   // for writeLogFile
#include <EEPROM.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern int currentState;
extern int Decimal_CoverageDuration;
extern int Counter_FailGPS;
extern int Counter_FailWIFI;

void initializeEEPROM() {
  int storedValue = EEPROM.read(EE_ADDR_STATE);

  if (storedValue == 0xFF) {  // EEPROM not yet initialised
    writeLogFile("EEPROM no ini., saving default value.");
    EEPROM.write(EE_ADDR_STATE, 0);
    EEPROM.commit();
    storedValue = 6;
  }

  currentState = storedValue;
}

// ------- State (addr 0) -------
void eepromSaveState(int newstate) {
  EEPROM.write(EE_ADDR_STATE, newstate);
  EEPROM.commit();
  delay(10);
}

void eepromInitState() {
  EEPROM.write(EE_ADDR_STATE, 0);
  EEPROM.commit();
  delay(10);
  currentState = EEPROM.read(EE_ADDR_STATE);
}

void changeStateTo(int newState) {
  currentState = newState;
  eepromSaveState(currentState);
}

int eepromReadState() {
  return EEPROM.read(EE_ADDR_STATE);
}

// ------- Coverage state (addr 1) -------
void SetCoverageStateTo(int NewCoverageState) {
  EEPROM.write(EE_ADDR_COVERAGE_STATE, NewCoverageState);
  EEPROM.commit();
  delay(50);
}

int eepromReadCoverageState() {
  return EEPROM.read(EE_ADDR_COVERAGE_STATE);
}

// ------- Coverage duration (addr 2-3, big-endian) -------
void eepromSaveTimeCoverage(int timeCoverage) {
  EEPROM.write(EE_ADDR_COVERAGE_DUR_HI, (timeCoverage >> 8) & 0xFF);
  EEPROM.write(EE_ADDR_COVERAGE_DUR_LO, timeCoverage & 0xFF);
  EEPROM.commit();
  delay(50);
}

void SetCoverageDurationTo_0() {
  Decimal_CoverageDuration = 0;
  eepromSaveTimeCoverage(Decimal_CoverageDuration);
}

int eepromReadCoverageDuration() {
  return (EEPROM.read(EE_ADDR_COVERAGE_DUR_HI) << 8) | EEPROM.read(EE_ADDR_COVERAGE_DUR_LO);
}

// ------- GPS-fail counter (addr 4) -------
void eepromSaveCounterGPSFail(int counter) {
  EEPROM.write(EE_ADDR_COUNTER_GPS_FAIL, counter);
  EEPROM.commit();
  delay(50);
}

void SetCounterFailGPSTo_0() {
  Counter_FailGPS = 0;
  eepromSaveCounterGPSFail(Counter_FailGPS);
}

int eepromReadCounterGPSFail() {
  return EEPROM.read(EE_ADDR_COUNTER_GPS_FAIL);
}

// ------- WiFi-fail counter (addr 5) -------
void eepromSaveCounterWIFIFail(int counter) {
  EEPROM.write(EE_ADDR_COUNTER_WIFI_FAIL, counter);
  EEPROM.commit();
  delay(50);
}

void SetCounterFailWIFITo_0() {
  Counter_FailWIFI = 0;
  eepromSaveCounterWIFIFail(Counter_FailWIFI);
}

void IncrementCounterFailWIFI() {
  if (Counter_FailWIFI >= 2) {  // Means needs to start from 0
    Counter_FailWIFI = 0;
  } else {
    Counter_FailWIFI = Counter_FailWIFI + 1;
  }
  eepromSaveCounterWIFIFail(Counter_FailWIFI);
}

int eepromReadCounterWIFIFail() {
  return EEPROM.read(EE_ADDR_COUNTER_WIFI_FAIL);
}

// ------- Synctime (addr 6) -------
void eepromSaveSyncTime(int syncTime) {
  EEPROM.write(EE_ADDR_SYNCTIME, syncTime);
  EEPROM.commit();
}

int eepromReadSyncTime() {
  return EEPROM.read(EE_ADDR_SYNCTIME);
}
