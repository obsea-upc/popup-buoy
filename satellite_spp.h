#pragma once
#include <Arduino.h>
#include "previpass.h"   // for AopSatelliteEntry_t

// Satellite Pass Prediction (SPP) over the locally stored AOP table.

// Shared by surface states 4, 5 and 6: predicts the next pass (or handles a GPS
// failure) and sets the global secondsBeforeNextStatellite + EEPROM coverage flags.
// lowPower=true is the critical-battery variant (state 5).
void runSatellitePassPrediction(bool lowPower);

// Call after the fix on a wake with coverage: light-sleeps whatever is left of
// TIME_LESS_BEFORE_AWAKENING, so the first message goes out as the session opens
// rather than as soon as the fix is in. Does nothing on a wake into a session
// that is already running (the overlap restart) or when the fix used it all.
void sppHoldUntilSessionStart();

// Seconds until the next transmit session worth waking for, and stores that
// session's length as the coverage duration in EEPROM. A session is one or more
// passes close enough together to be served by a single wake; sessions that
// cannot carry SPP_MIN_DATA_MSGS messages are not offered at all. Negative means
// the session has already started, which the caller reuses (the overlap branch).
int NextSatellite(double &gpsLat, double &gpsLong, AopSatelliteEntry_t *aopTable, uint8_t nbSatsInAopTable, float MinElev);

// ---- Per-message attribution ----------------------------------------------
// Which satellite is overhead right now and how high, so every transmission can
// be logged with the geometry it went out under. Joined against the CLS export
// by timestamp, this is what turns a campaign into a reception-vs-elevation
// dataset instead of a pile of counts.

struct SppTxContext_t {
  char     satName[4];   //!< AOP entry name, exact - not recovered from the 6-bit id
  uint8_t  elevNow;      //!< interpolated elevation at this instant, degrees
  uint8_t  elevMax;      //!< the pass maximum, as predicted
  uint16_t sinceStart;   //!< seconds since this pass came above MinElev
  uint16_t passDur;      //!< the pass total, seconds above MinElev
};

// Caches the passes covering the wake that is starting. Call once per wake,
// after the GPS fix and before transmitting; cheap, because the window is the
// session rather than a day. Safe to call when there is no coverage.
void sppBeginSession(double lat, double lon, float minElev, int coverageDuration);

// Fills out with the pass containing nowUnix. False when nothing is overhead,
// which is normal for the recovery messages sent outside any coverage.
bool sppTxContext(uint32_t nowUnix, SppTxContext_t &out);

// AOP table (/AOP.txt) parsing/printing.
void parseLine(const String &line, AopSatelliteEntry_t &data);
void readSatelliteData(AopSatelliteEntry_t *aopTable, uint8_t &nbSatsInAopTable);
void printAopTable(const AopSatelliteEntry_t *aopTable, uint8_t nbSatsInAopTable);

// ---- The session's passes, for scheduling ---------------------------------
// Valid after sppBeginSession(). The passes are in time order.

// How many passes this wake covers. Zero when there is no coverage, or when the
// AOP table could not be read.
uint8_t sppSessionPassCount();

// Peak (midpoint) and end of pass `index`, as unix seconds on the RTC's clock,
// plus its maximum elevation. False when the index is out of range.
bool sppSessionPass(uint8_t index, uint32_t &peakUnix, uint32_t &endUnix, uint8_t &elevMax);
