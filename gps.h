#pragma once
#include <Arduino.h>

// Open the GPS serial port (larger RX buffer than the library default, see gps.cpp).
void gpsSerialBegin();

// Power the GPS up or down, port included. V2 only: there the GPS has its own
// switch and is on only while a fix is being taken. On a V1 the GPS shares the
// satellite module's rail, which the sketch manages, so these do nothing.
void gpsPowerOn();
void gpsPowerOff();

// V2: set the sea dynamic model and AssistNow Autonomous, both in the
// receiver's RAM, so at every power-up. V1: its clone receiver takes no
// configuration; only a debug probe runs, with GPS_DEBUG_NMEA_GSA.
void configGPS();

// Quick check (30 s) for any satellite in view. Returns true on first satellite seen.
bool gpsAcquireSatellites();

// Acquire a full GPS fix (lat/long/date/time/epoch) until fix or maxGPSTimeout.
// Test aid: with TEST_FORCE_GPS_VILANOVA_PB2 (on by default), tapping PB_2 injects a Vilanova fix from the RTC.
void gpsAcquireData(double &gpsLat, double &gpsLong, uint16_t &gpsYear, uint8_t &gpsMonth, uint8_t &gpsDay, uint8_t &gpsHour, uint8_t &gpsMinute, uint8_t &gpsSecond, uint32_t &epochTime, bool &gpsFix);

// Persist the fix to the SD track file, or (on no fix) mark lat/long invalid and take epoch from the RTC.
void gpsSave(double &gpsLat, double &gpsLong, uint16_t &gpsYear, uint8_t &gpsMonth, uint8_t &gpsDay, uint8_t &gpsHour, uint8_t &gpsMinute, uint8_t &gpsSecond, uint32_t &epochTime, bool &gpsFix);

// Append one CSV row (lat,long,Y,M,D,h,m,s;) to the GPS track file on the SD card.
bool saveGPStoSD(double &gpsLat, double &gpsLong, uint16_t &gpsYear, uint8_t &gpsMonth, uint8_t &gpsDay, uint8_t &gpsHour, uint8_t &gpsMinute, uint8_t &gpsSecond);

// --- GPS fix data (defined in gps.cpp; shared with the sketch and the satellite modules) ---
extern double gpsLat, gpsLong;
extern uint8_t gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond;
extern uint16_t gpsYear;
extern bool gpsFix;
extern uint32_t epochTime;
extern int maxGPSTimeout;  // GPS acquisition timeout (ms), loaded from conf.txt

// GPS track file path (defined in gps.cpp).
extern const char *GPSfilename;
