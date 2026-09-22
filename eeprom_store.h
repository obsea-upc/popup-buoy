#pragma once
#include <Arduino.h>

// ------- EEPROM address map (EEPROM_SIZE = 7 bytes, see conf.h) -------
#define EE_ADDR_STATE             0  // state machine state (0..6)
#define EE_ADDR_COVERAGE_STATE    1  // 1 = a satellite pass is coming, 0 = none
#define EE_ADDR_COVERAGE_DUR_HI   2  // coverage duration (seconds), high byte
#define EE_ADDR_COVERAGE_DUR_LO   3  // coverage duration (seconds), low byte
#define EE_ADDR_COUNTER_GPS_FAIL  4  // consecutive GPS-fix failures
#define EE_ADDR_COUNTER_WIFI_FAIL 5  // consecutive WiFi failures
#define EE_ADDR_SYNCTIME          6  // synctime hour
#define EE_ADDR_DATA_DONE         7  // 1 = the seabed data file has been sent to the end

// Initialisation
void initializeEEPROM();

// State (addr 0)
void eepromSaveState(int newstate);
void eepromInitState();
void changeStateTo(int newState);
int  eepromReadState();

// Coverage state (addr 1)
void SetCoverageStateTo(int NewCoverageState);
int  eepromReadCoverageState();

// Coverage duration (addr 2-3, big-endian)
void eepromSaveTimeCoverage(int timeCoverage);
void SetCoverageDurationTo_0();
int  eepromReadCoverageDuration();

// GPS-fail counter (addr 4)
void eepromSaveCounterGPSFail(int counter);
void SetCounterFailGPSTo_0();
int  eepromReadCounterGPSFail();

// WiFi-fail counter (addr 5)
void eepromSaveCounterWIFIFail(int counter);
void SetCounterFailWIFITo_0();
void IncrementCounterFailWIFI();
int  eepromReadCounterWIFIFail();

// Synctime (addr 6)
void eepromSaveSyncTime(int syncTime);
int  eepromReadSyncTime();

// Data-file-exhausted flag (addr 7)
void eepromSaveDataDone(bool done);
bool eepromReadDataDone();
