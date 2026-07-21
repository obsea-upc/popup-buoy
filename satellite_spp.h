#pragma once
#include <Arduino.h>
#include "previpass.h"   // for AopSatelliteEntry_t

// Satellite Pass Prediction (SPP) over the locally stored AOP table.

// Shared by surface states 4, 5 and 6: predicts the next pass (or handles a GPS
// failure) and sets the global secondsBeforeNextStatellite + EEPROM coverage flags.
// lowPower=true is the critical-battery variant (state 5).
void runSatellitePassPrediction(bool lowPower);

// Seconds until the earliest next satellite pass for the given position; also stores
// the pass duration as the coverage duration in EEPROM.
int NextSatellite(double &gpsLat, double &gpsLong, AopSatelliteEntry_t *aopTable, uint8_t nbSatsInAopTable, float MinElev);

// AOP table (/AOP.txt) parsing/printing.
void parseLine(const String &line, AopSatelliteEntry_t &data);
void readSatelliteData(AopSatelliteEntry_t *aopTable, uint8_t &nbSatsInAopTable);
void printAopTable(const AopSatelliteEntry_t *aopTable, uint8_t nbSatsInAopTable);
