#include "satellite_spp.h"
#include "conf.h"
#include "logging.h"
#include "eeprom_store.h"
#include "power_sleep.h"
#include "previpass.h"
#include "previpass_util.h"
#include <RTClib.h>
#include <SD.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern RTC_DS3231 rtcExt;

// AOP table file, owned by this module (internal).
const char *AOPfilename = "/AOP.txt";
File AOPFile;
extern int Decimal_CoverageDuration;
extern String messageLogFile;
extern double gpsLat, gpsLong;   // globals (NextSatellite's same-named params shadow these inside it)
extern bool gpsFix;
extern float MinElev;
extern int secondsBeforeNextStatellite;
extern int Counter_FailGPS;
extern int sleeptime_errorGPS_s;
extern int sleeptime_errorGPS_recurrent_s;

// ---------------------------------------------------------------------------
// Pass planning
//
// The old prediction asked each satellite for its next pass and kept the
// earliest, one wake per pass, and discarded anything shorter than
// minPassDurationMinute. That threw away two different things: a short pass
// sitting next to another one, which is worth having because both fit in a
// single wake, and the second half of an overlap, which needed its own error
// branch to recover.
//
// This builds the whole day instead: every pass above MinElev, grouped into
// sessions wherever the gap is cheaper to stay awake through than to sleep and
// wake for, and then the first session that actually pays for its wake.
// ---------------------------------------------------------------------------

struct SppPass {
  uint32_t start;   //!< Unix seconds, same base as the RTC
  uint16_t dur;     //!< seconds above MinElev
  uint8_t  elev;    //!< maximum elevation, degrees
  uint8_t  sat;     //!< index into the AOP table - exact, never recovered from satHexId
};

static SppPass  sppPasses[SPP_MAX_PASSES];
static uint16_t sppNbPasses = 0;

// Messages a wake of this length yields: the GPS fix comes out of the front, the
// rest is divided by the transmit spacing actually in use.
static int sppMessagesIn(long windowSeconds) {
  const long usable = windowSeconds - SPP_GPS_FIX_S;
  const int  cycle  = INTERVAL_MS / 1000;
  if (usable <= 0 || cycle <= 0) return 0;
  return (int)(usable / cycle);
}

// Every pass of every satellite in the configured window, sorted by start time.
// Asked one satellite at a time on purpose: the returned id is a 6-bit field and
// cannot name a satellite once the legacy birds are in the table, so the only
// reliable attribution is which entry we asked about. The library resets its
// pool on each call, so each list is copied out before the next one.
static void sppCollectPasses(struct PredictionPassConfiguration_t *cfg,
                             AopSatelliteEntry_t *aopTable, uint8_t nbSats) {
  sppNbPasses = 0;

  for (uint8_t i = 0; i < nbSats; i++) {
    bool overflow = false;
    struct SatPassLinkedListElement_t *list =
        PREVIPASS_compute_new_prediction_pass_times(cfg, &aopTable[i], 1, &overflow);
    if (overflow) {
      writeLogFile("SPP: pass pool overflow on " + String(aopTable[i].entryName));
    }
    for (struct SatPassLinkedListElement_t *p = list; p != NULL; p = p->next) {
      if (sppNbPasses >= SPP_MAX_PASSES) break;
      sppPasses[sppNbPasses].start = p->element.epoch;
      sppPasses[sppNbPasses].dur   = (uint16_t)p->element.duration;
      sppPasses[sppNbPasses].elev  = (uint8_t)p->element.elevationMax;
      sppPasses[sppNbPasses].sat   = i;
      sppNbPasses++;
    }
    if (sppNbPasses >= SPP_MAX_PASSES) {
      writeLogFile("SPP: pass list full at " + String(SPP_MAX_PASSES) + ", ignoring the rest");
      break;
    }
  }

  // Insertion sort by start time. A day is at most a few hundred entries and
  // they arrive nearly grouped, so this is cheaper than it looks.
  for (uint16_t i = 1; i < sppNbPasses; i++) {
    SppPass k = sppPasses[i];
    int16_t j = (int16_t)i - 1;
    while (j >= 0 && sppPasses[j].start > k.start) {
      sppPasses[j + 1] = sppPasses[j];
      j--;
    }
    sppPasses[j + 1] = k;
  }
}

void runSatellitePassPrediction(bool lowPower) {
  if (gpsFix) {

    SetCounterFailGPSTo_0();  //if we have fixed the gps, set counter to zero cause the counter is valid for consecutive fails

    AopSatelliteEntry_t aopTable[maxAOPSize];
    uint8_t nbSatsInAopTable = maxAOPSize;

    readSatelliteData(aopTable, nbSatsInAopTable);
    #ifdef SERIAL_DEBUG
      printAopTable(aopTable, nbSatsInAopTable);
    #endif
    if (lowPower) {
      MinElev = critMinElev;
    }
    bool SPP_progress = true;
    int sppAttempts = 0;   // bounds the retry loop below

    while (SPP_progress){

      secondsBeforeNextStatellite = NextSatellite(gpsLat, gpsLong, aopTable, nbSatsInAopTable, MinElev);
      secondsBeforeNextStatellite -= TIME_LESS_BEFORE_AWAKENING;  // Used to awake before being in range of the satellite
      SetCoverageStateTo(1);                                     // There is coverage so we put it in EEPROM
      // no need to save coverage duration, saved inside the funtion NextSatellite
      if (secondsBeforeNextStatellite > 0) {
        SPP_progress=false;
      }
      // --- HANDLING THE OVERLAPPING AND THE SPP ERRORS ---
      // Reuse a pass that is already running, provided enough of it is left to be
      // worth restarting the cycle for - see SPP_MIN_USABLE_COVERAGE_S in conf.h
      // for why that bar is deliberately not one message.
      if (secondsBeforeNextStatellite <= 0 && Decimal_CoverageDuration + secondsBeforeNextStatellite > SPP_MIN_USABLE_COVERAGE_S) {  // To be able to use the current coverage
        Decimal_CoverageDuration += secondsBeforeNextStatellite;                                             // To get the duration left on the coverage
        eepromSaveTimeCoverage(Decimal_CoverageDuration);
        writeLogFile("SPP is overlapping, starting state " + String(lowPower ? 5 : 4) + " again.");
        changeStateTo(lowPower ? 5 : 4);
        delay(10);
        secondsBeforeNextStatellite = lowPower ? 0 : 5;      //go directly to the state to continue transmitting, no sleep
        SPP_progress=false;

      } else if (secondsBeforeNextStatellite <= 0) {  // If we are not in the case of overlapping coverage but only with a coverage which is over
        if (lowPower) {
          secondsBeforeNextStatellite = CRIT_FACTOR*3600; //in lowlevel mode, no need to repeat the SPP in SPP error, sleep 3 hours
          SetCoverageDurationTo_0();
          SetCoverageStateTo(0);
          writeLogFile("SPP ERROR. Sleeping for 3 h and starting state 5 again with no satellite.");
          SPP_progress=false;
        } else {
          sppAttempts++;
          if (sppAttempts >= SPP_MAX_RETRIES) {
            // Stop spinning. On 9 Aug both buoys spent over five hours here -
            // awake, light-sleeping 60 s at a time and recomputing a prediction
            // that could not succeed - which is a third of the test burned for
            // nothing. A deep sleep costs almost no power and comes back with a
            // fresh GPS fix, which is exactly what the prediction was missing.
            secondsBeforeNextStatellite = SPP_GIVEUP_SLEEP_S;
            SetCoverageDurationTo_0();
            SetCoverageStateTo(0);
            writeLogFile("SPP ERROR x" + String(sppAttempts) + ". Giving up and sleeping "
                         + String(SPP_GIVEUP_SLEEP_S) + " s for a fresh GPS fix.");
            SPP_progress=false;
          } else {
            Decimal_CoverageDuration = 60;  // I found some problems so let's just sleep for 1 minute and repeat the SPP
            SetCoverageStateTo(0);
            writeLogFile("SPP ERROR " + String(sppAttempts) + "/" + String(SPP_MAX_RETRIES)
                         + ". Sleeping light for " + String(Decimal_CoverageDuration) + " s and repeating SPP.");
            goToSleep(Decimal_CoverageDuration);
            SPP_progress=true;
          }
        }
      }
    }
  } else {  // If GPS is not fixed

    Counter_FailGPS = eepromReadCounterGPSFail(); // How many times in a row the GPS has not been fixed?
    writeLogFile("Failed to fix GPS, no data was stored to SD");
    SetCoverageDurationTo_0();  // if we don't get the position it's better to keep in memory that coverage is null so that if we get GPS in the next state 4 we do not send messages if we don't if there is a satellite
    SetCoverageStateTo(0);
    Counter_FailGPS += 1;       // each time the gps can't find the data, the counter increase of 1
    eepromSaveCounterGPSFail(Counter_FailGPS);

    int critFactor = lowPower ? CRIT_FACTOR : 1;  // battery-critical (state 5) uses longer sleeps
    if (Counter_FailGPS > 0 && Counter_FailGPS < 3) {  //When the buoy fail less than 3 times in a row, the sleeping time is shorter
      secondsBeforeNextStatellite = sleeptime_errorGPS_s * critFactor;
      writeLogFile("GPS failing : counter = " + String(Counter_FailGPS));
    } else if (Counter_FailGPS >= 3) {                                  // 3rd GPS failing, counter goes to 0 and sleep for 1 hour
      secondsBeforeNextStatellite = sleeptime_errorGPS_recurrent_s * critFactor;  //In reality we will never sleep 1h becaule it will be later set to a maximum sleep of 20 minutes, so cycle will be 3, 3, 20
      SetCounterFailGPSTo_0();
      writeLogFile("GPS failing : counter = 3 ");
    }

  }
}

int NextSatellite(double &gpsLat, double &gpsLong, AopSatelliteEntry_t *aopTable, uint8_t nbSatsInAopTable, float MinElev) {

  DateTime now = rtcExt.now();

  uint16_t gpsYear = now.year();
  uint8_t gpsMonth = now.month();
  uint8_t gpsDay = now.day();
  uint8_t gpsHour = now.hour();
  uint8_t gpsMinute = now.minute();
  uint8_t gpsSecond = now.second();

  int nextDay = gpsDay;
  int nextMonth = gpsMonth;
  int nextYear = gpsYear;

  if (nextDay == 30 || nextDay == 31) {
    nextDay = 1;
    nextMonth += 1;
    if (nextMonth >= 12) {
      nextMonth = 1;
      nextYear += 1;
    }
  } else {
    nextDay += 1;
  }

  writeLogFile("Min. elevation set to: " + String(MinElev));

  struct PredictionPassConfiguration_t prepasConfiguration = {
    gpsLat,                                                           //< Geodetic latitude of the beacon (deg.) [-90, 90]
    gpsLong,                                                          //< Geodetic longitude of the beacon (deg.E)[0, 360]
    { gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond },     //< Beginning of prediction (Y/M/D, hh:mm:ss)
    { nextYear, nextMonth, nextDay, gpsHour, gpsMinute, gpsSecond },  //< End of prediction (Y/M/D, hh:mm:ss)
    MinElev,                                                             //< Minimum elevation of passes [0, 90](default 5 deg)
    90.0f,                                                            //< Maximum elevation of passes  [maxElevation >=
                                                                      //< minElevation] (default 90 deg)
    // No duration filter. It used to be 3 minutes, which quietly did two jobs:
    // it kept the buoy from waking for a pass too short to be worth it, and it
    // hid short passes from the prediction entirely. Only the first is wanted.
    // A 100 s pass next to another one is worth having - both fit in one wake -
    // and dropping it here meant the planner could never see the pair. The
    // "is this wake worth making" test now lives on the session, below, where
    // it can tell an isolated short pass from a stacked one.
    0.0f,                                                             //< Minimum duration (default 5 minutes)
    1000,                                                             //< Maximum number of passes per satellite (default
                                                                      //< 1000)
    5,                                                                //< Linear time margin (in minutes/6months) (default
                                                                      //< 5 minutes/6months)
    // 10 s, not the 30 s default. At 30 s the sampling grid is too coarse to
    // measure a pass properly and the answer depends on when you ask: the same
    // KINEIS-5E pass of 10 Aug came out as 276 s at 87 deg from one window and
    // 96 s at 60 deg from another. The short reading then fell under
    // minPassDurationMinute and the pass was dropped entirely - which is how a
    // near-zenith pass went missing while the buoy slept toward a 39 deg one.
    // Measured on hardware: 10 s and 5 s agree exactly and 1 s confirms within
    // 4 s and 1 deg. 5 s is used rather than 10 because the library jumps ahead
    // 20 steps while the satellite is still far away, and at 10 s that jump is
    // 200 s - longer than the shortest passes worth having. At 5 s it is 100 s.
    // Costs 53 ms per satellite against 9; irrelevant next to losing a pass.
    5                                                                //< Computation step
  };

  sppCollectPasses(&prepasConfiguration, aopTable, nbSatsInAopTable);

  if (sppNbPasses == 0) {
    // Nothing above MinElev in the whole prediction window. Report it as a
    // negative time with zero coverage so the caller takes its error path, which
    // is now bounded, instead of us inventing a schedule out of an empty result.
    writeLogFile("SPP: no satellite pass found above " + String(MinElev) + " deg");
    SetCoverageDurationTo_0();
    return -1;
  }

  // Group the passes into sessions and take the first one that pays for its wake.
  // A pass that has already ENDED is skipped - it can only come back as a
  // negative time and be thrown away. One still in progress is kept: the caller
  // reuses what is left of it, which is what the overlap branch is for.
  const uint32_t nowUnix = now.unixtime();

  uint32_t sessionStart = 0, sessionEnd = 0;
  uint16_t sessionFirst = 0, sessionCount = 0;
  uint16_t sessionsSeen = 0, sessionsSkipped = 0;
  bool     sessionFound = false;

  for (uint16_t i = 0; i < sppNbPasses; ) {
    uint32_t start = sppPasses[i].start;
    uint32_t end   = start + sppPasses[i].dur;
    uint16_t first = i;
    uint16_t count = 1;

    // Absorb the following passes while the gap is cheaper to stay awake through
    // than to sleep and wake for. A negative gap is an overlap and needs no
    // special case here, which is the branch the old code had to carry.
    uint16_t j = i + 1;
    while (j < sppNbPasses && (long)sppPasses[j].start - (long)end <= SPP_MERGE_GAP_S) {
      const uint32_t e = sppPasses[j].start + sppPasses[j].dur;
      if (e > end) end = e;
      count++;
      j++;
    }
    i = j;

    if (end <= nowUnix) continue;            // already over
    sessionsSeen++;

    bool worthIt;
    if (start <= nowUnix) {
      // Already under way. No wake to decide on - the question is only whether
      // enough is left to be worth restarting the transmit cycle, which is the
      // same bar the caller applies in its overlap branch. Checking it here as
      // well means a session too far gone is passed over for the next one
      // instead of being handed back to be rejected and retried.
      worthIt = (long)(end - nowUnix) >= SPP_MIN_USABLE_COVERAGE_S;
    } else {
      // Matias's rule: stacked short passes are worth a wake, an isolated one is
      // not. Measured over the whole session, so the two cases separate
      // themselves without the planner having to tell them apart.
      worthIt = sppMessagesIn((long)(end - start)) >= SPP_MIN_DATA_MSGS;
    }
    if (!worthIt) sessionsSkipped++;

    const bool takeThis = worthIt && !sessionFound;
    if (takeThis) {
      sessionStart = start;
      sessionEnd   = end;
      sessionFirst = first;
      sessionCount = count;
      sessionFound = true;
    }

    // The whole day is enumerated even after the answer is found. It costs a few
    // dozen more iterations of arithmetic, it makes the counts logged below
    // describe the day rather than only the part before the first hit, and it is
    // what the dump prints. Serial only - a session table on every wake would
    // bury the SD log, and this disappears entirely without SERIAL_DEBUG.
    #ifdef SPP_DUMP_PLAN
      if (sessionsSeen == 1) {
        SerialPrintDebugln("SPP plan (" + String(sppNbPasses) + " passes above "
                           + String(MinElev) + " deg):");
        SerialPrintDebugln("   when      dur  msgs  what");
      }
      {
        struct CalendarDateTime_t d;
        PREVIPASS_UTIL_date_stu90_calendar(start - EPOCH_90_TO_70_OFFSET, &d);
        String row = String(takeThis ? " -> " : "    ")
                   + (d.gpsHour < 10 ? "0" : "") + String(d.gpsHour) + ":"
                   + (d.gpsMinute < 10 ? "0" : "") + String(d.gpsMinute) + ":"
                   + (d.gpsSecond < 10 ? "0" : "") + String(d.gpsSecond)
                   + "  " + String(end - start) + "s  "
                   + String(sppMessagesIn((long)(end - start))) + "  ";
        for (uint16_t k = 0; k < count; k++) {
          row += String(aopTable[sppPasses[first + k].sat].entryName) + "("
               + String(sppPasses[first + k].elev) + ") ";
        }
        if (!worthIt) row += " [too small]";
        SerialPrintDebugln(row);
      }
    #endif
  }

  if (!sessionFound) {
    // Every session in the next day is too small to be worth the wake. This is
    // not a prediction failure, so it must not take the error path: report a
    // long time with no coverage and let the caller cap it at max_sleep_time_s,
    // which is exactly the recovery-message behaviour that already exists.
    writeLogFile("SPP: " + String(sessionsSeen) + " sessions above " + String(MinElev)
                 + " deg, none carries " + String(SPP_MIN_DATA_MSGS)
                 + " messages. Nothing worth waking for.");
    SetCoverageDurationTo_0();
    return 24 * 3600;
  }

  // Name every satellite in the chosen session. Bounded: a busy session can hold
  // several passes and building this with strcat into a fixed buffer overran it.
  char sats[64];
  sats[0] = '\0';
  size_t used = 0;
  for (uint16_t k = 0; k < sessionCount; k++) {
    const SppPass &pp = sppPasses[sessionFirst + k];
    const int w = snprintf(sats + used, sizeof(sats) - used, "%s(%u) ",
                           aopTable[pp.sat].entryName, (unsigned)pp.elev);
    if (w <= 0 || (size_t)w >= sizeof(sats) - used) break;
    used += (size_t)w;
  }

  writeLogFile("SPP: " + String(sppNbPasses) + " passes above " + String(MinElev)
               + " deg, " + String(sessionsSeen) + " sessions, " + String(sessionsSkipped)
               + " too small. Taking " + String(sessionCount) + " pass(es): " + String(sats));

  // Plain locals, not SatelliteNextPassPrediction_t: its duration is a 12-bit
  // field and a merged session can outrun 4095 s at a low elevation floor, which
  // would truncate without saying so.
  const uint32_t sessionEpoch    = sessionStart;
  const uint32_t sessionDuration = sessionEnd - sessionStart;
  const uint8_t  earliestIdx     = sppPasses[sessionFirst].sat;

  uint8_t sessionElevMax = 0;
  for (uint16_t k = 0; k < sessionCount; k++) {
    if (sppPasses[sessionFirst + k].elev > sessionElevMax) {
      sessionElevMax = sppPasses[sessionFirst + k].elev;
    }
  }

  //! Sat name, taken from the AOP entry that actually produced this pass.
  //
  // Not looked up from the returned id, because that id cannot identify a
  // satellite on its own: SatelliteNextPassPrediction_t stores it in a 6-bit
  // field (previpass.h, "[0x01..0x3F]"), so everything above 0x3F comes back
  // truncated. Masking to 6 bits appeared to work in August only because the AOP
  // then held Kineis satellites alone. With the legacy ones loaded the low bits
  // collide - 2A (0x31) against CS (0xF1) both give 49, and 5C (0xBB) against
  // MC (0xFB) both give 59 - and passes come out labelled with the wrong
  // satellite. Measured against a CLS export on the 3 Sep AOP: twelve of 144.
  //
  // The loop above already asks one satellite at a time, so the answer is just
  // which entry won. No lookup, no ambiguity, correct for any fleet.
  char satNameTwoChars[sizeof(aopTable[0].entryName)];
  strncpy(satNameTwoChars, aopTable[earliestIdx].entryName, sizeof(satNameTwoChars) - 1);
  satNameTwoChars[sizeof(satNameTwoChars) - 1] = '\0';

  struct CalendarDateTime_t viewable_timedata;

  PREVIPASS_UTIL_date_stu90_calendar(sessionEpoch - EPOCH_90_TO_70_OFFSET,
                                     &viewable_timedata);




  String response = "Data: " + String(viewable_timedata.gpsDay) + "/" + String(viewable_timedata.gpsMonth) + "/" + String(viewable_timedata.gpsYear) + ".  The next satellite will be "
  + String(satNameTwoChars) +     " at " + String(viewable_timedata.gpsHour) + ":" + String(viewable_timedata.gpsMinute) + ":" + String(viewable_timedata.gpsSecond) + "UTC, with a duration of "
  + String((int)sessionDuration / 60) + " min and "  + String((int)sessionDuration % 60) + " sec and a maximum elevation of " + String((int)sessionElevMax) + "º"
  + ", carrying " + String(sppMessagesIn((long)sessionDuration)) + " messages";
  writeLogFile(response);


  DateTime now3 = rtcExt.now();

  DateTime compareTime = DateTime(viewable_timedata.gpsYear, viewable_timedata.gpsMonth, viewable_timedata.gpsDay, viewable_timedata.gpsHour, viewable_timedata.gpsMinute, viewable_timedata.gpsSecond);

  // calculate difference between the 2 times in seconds
  int diff = compareTime.unixtime() - now3.unixtime();

  SerialPrintDebug("Time now: ");
  SerialPrintDebugln(now3.timestamp(DateTime::TIMESTAMP_FULL));
  SerialPrintDebug("Time SPP: ");
  SerialPrintDebugln(compareTime.timestamp(DateTime::TIMESTAMP_FULL));
  SerialPrintDebug("Difference in seconds: ");
  SerialPrintDebugln(diff);


  // -------To remember the time duration in EEPROM---------)
  Decimal_CoverageDuration = (int)sessionDuration;
  eepromSaveTimeCoverage(Decimal_CoverageDuration);
  Decimal_CoverageDuration = eepromReadCoverageDuration();
  SerialPrintDebugln(" Time of coverage next satellite : " + String(Decimal_CoverageDuration));

  messageLogFile = "Next Satelitte : Time before next satellite :" + String(diff) + " sec and coverage : " + String(Decimal_CoverageDuration) + String(" sec");
  writeLogFile(messageLogFile);

  return diff;
}

// ---------------------------------------------------------------------------
// Per-message attribution
//
// The prediction that chose this wake ran a cycle ago, before a deep sleep, so
// nothing of it survives in RAM. Rather than carry it through EEPROM, the passes
// covering this wake are recomputed once when the wake starts. The window is the
// session rather than a day, so it costs a few milliseconds, and it has a second
// use beyond logging: it is an independent check, made with a fresh GPS fix and
// the current clock, that a satellite really is up.
// ---------------------------------------------------------------------------

#define SPP_SESSION_MAX_PASSES 8
// How far either side of the expected session to look. The wake starts
// TIME_LESS_BEFORE_AWAKENING early and the GPS fix eats into the front, so the
// window has to reach back before "now" to catch a pass already in progress.
#define SPP_SESSION_MARGIN_S 300

static SppPass sppSessionPasses[SPP_SESSION_MAX_PASSES];
static uint8_t sppSessionNbPasses = 0;
static char    sppSessionNames[SPP_SESSION_MAX_PASSES][sizeof(((AopSatelliteEntry_t *)0)->entryName)];
static float   sppSessionMinElev = 0.0f;

void sppBeginSession(double lat, double lon, float minElev, int coverageDuration) {
  sppSessionNbPasses = 0;
  sppSessionMinElev  = minElev;

  if (coverageDuration <= 0) return;   // no coverage this wake: recovery messages only

  // Static, not on the stack: thirty AOP entries are about 1.4 kB and this runs
  // deep inside the transmit path rather than at the top of a state.
  static AopSatelliteEntry_t aopTable[maxAOPSize];
  uint8_t nbSatsInAopTable = maxAOPSize;
  readSatelliteData(aopTable, nbSatsInAopTable);
  if (nbSatsInAopTable == 0) {
    writeLogFile("SPP session: AOP table empty, transmissions will not be attributed");
    return;
  }

  DateTime now = rtcExt.now();
  struct CalendarDateTime_t from, to;
  PREVIPASS_UTIL_date_stu90_calendar(
      (now.unixtime() - SPP_SESSION_MARGIN_S) - EPOCH_90_TO_70_OFFSET, &from);
  PREVIPASS_UTIL_date_stu90_calendar(
      (now.unixtime() + (uint32_t)coverageDuration + SPP_SESSION_MARGIN_S) - EPOCH_90_TO_70_OFFSET, &to);

  struct PredictionPassConfiguration_t cfg = {
    lat, lon,
    { from.gpsYear, from.gpsMonth, from.gpsDay, from.gpsHour, from.gpsMinute, from.gpsSecond },
    { to.gpsYear,   to.gpsMonth,   to.gpsDay,   to.gpsHour,   to.gpsMinute,   to.gpsSecond   },
    minElev, 90.0f, 0.0f, 1000, 5, 5
  };

  sppCollectPasses(&cfg, aopTable, nbSatsInAopTable);

  // Copy out of the shared list: NextSatellite runs later in this same wake and
  // rebuilds sppPasses for the whole day, which would otherwise clobber this.
  for (uint16_t i = 0; i < sppNbPasses && sppSessionNbPasses < SPP_SESSION_MAX_PASSES; i++) {
    sppSessionPasses[sppSessionNbPasses] = sppPasses[i];
    strncpy(sppSessionNames[sppSessionNbPasses], aopTable[sppPasses[i].sat].entryName,
            sizeof(sppSessionNames[0]) - 1);
    sppSessionNames[sppSessionNbPasses][sizeof(sppSessionNames[0]) - 1] = '\0';
    sppSessionNbPasses++;
  }

  writeLogFile("SPP session: " + String(sppSessionNbPasses) + " pass(es) overhead this wake");
}

// Elevation at an instant. Interpolated, not computed: the library exposes no
// elevation-at-a-time call, only the maximum over a pass. A cosine between the
// threshold at the edges and the maximum at the centre is close enough for a
// covariate and is monotonic in the right direction. The raw ingredients go into
// the log next to it, so a better model can be fitted offline without having to
// repeat a campaign.
static float sppElevationAt(const SppPass &p, uint32_t nowUnix) {
  const float half = (float)p.dur / 2.0f;
  if (half <= 0.0f) return sppSessionMinElev;
  float u = fabsf((float)(nowUnix - p.start) - half) / half;
  if (u > 1.0f) u = 1.0f;
  const float e = sppSessionMinElev + ((float)p.elev - sppSessionMinElev) * cosf(u * (float)M_PI / 2.0f);
  return e < 0.0f ? 0.0f : e;
}

bool sppTxContext(uint32_t nowUnix, SppTxContext_t &out) {
  // The attribution floor is low enough that two satellites can be up at once.
  // Report the higher one: that is the pass most likely to have heard us, and
  // it is the one a reception model should be conditioned on.
  int8_t best = -1;
  float  bestElev = -1.0f;

  for (uint8_t i = 0; i < sppSessionNbPasses; i++) {
    const SppPass &p = sppSessionPasses[i];
    if (nowUnix < p.start || nowUnix >= (uint32_t)(p.start + p.dur)) continue;
    const float e = sppElevationAt(p, nowUnix);
    if (e > bestElev) { bestElev = e; best = (int8_t)i; }
  }

  if (best < 0) return false;

  const SppPass &p = sppSessionPasses[best];
  strncpy(out.satName, sppSessionNames[best], sizeof(out.satName) - 1);
  out.satName[sizeof(out.satName) - 1] = '\0';
  out.elevNow    = (uint8_t)bestElev;
  out.elevMax    = p.elev;
  out.sinceStart = (uint16_t)(nowUnix - p.start);
  out.passDur    = p.dur;
  return true;
}

void parseLine(const String &line, AopSatelliteEntry_t &data) { // Function to parse a line of data and fill the SatelliteData structure
    int dnlkStatus, uplkStatus;
    sscanf(line.c_str(), "%s %x %d %d %x %hu %hu %hu %hu %hu %hu %f %f %f %f %f %f",
           data.entryName, &data.satHexId, &data.satDcsId, &dnlkStatus, &uplkStatus,
           &data.bulletin.gpsYear, &data.bulletin.gpsMonth, &data.bulletin.gpsDay,
           &data.bulletin.gpsHour, &data.bulletin.gpsMinute, &data.bulletin.gpsSecond,
           &data.semiMajorAxisKm, &data.inclinationDeg, &data.ascNodeLongitudeDeg,
           &data.ascNodeDriftDeg, &data.orbitPeriodMin, &data.semiMajorAxisDriftMeterPerDay);

    data.downlinkStatus = (dnlkStatus == 0) ? SAT_DNLK_OFF : static_cast<SatDownlinkStatus_t>(dnlkStatus);
    #ifdef FORCE_A2_UPLINK_STATUS
        data.uplinkStatus = SAT_UPLK_ON_WITH_A2;
    #else
        data.uplinkStatus = (uplkStatus == 0) ? SAT_UPLK_OFF : static_cast<SatUplinkStatus_t>(uplkStatus);
    #endif
}

void readSatelliteData(AopSatelliteEntry_t *aopTable, uint8_t &nbSatsInAopTable) { // Function to read satellite data from a file
    if (!SD.begin(5)) {
        SerialPrintDebugln("Error al inicializar la tarjeta SD");
        return;
    }

    AOPFile = SD.open(AOPfilename, FILE_READ);

    if (!AOPFile) {
        SerialPrintDebugln("Error al abrir el archivo");
        return;
    }

    uint8_t dataIndex = 0;
    while (AOPFile.available() && dataIndex < maxAOPSize) {
        String line = AOPFile.readStringUntil('\n');
        parseLine(line, aopTable[dataIndex]);
        dataIndex++;
    }

    AOPFile.close();
    nbSatsInAopTable = dataIndex;
}

void printAopTable(const AopSatelliteEntry_t *aopTable, uint8_t nbSatsInAopTable) { // Function to print the AopSatelliteEntry_t array
    SerialPrintDebugln("struct AopSatelliteEntry_t aopTable[] = {");
    for (uint8_t i = 0; i < nbSatsInAopTable; i++) {
        Serial.printf("    { 0x%X, %d, %s, %s, {%hu, %hu, %hu, %hu, %hu, %hu}, %.3ff, %.3ff, %.3ff, %.3ff, %.3ff, %.3ff}, // %s\n",
                      aopTable[i].satHexId, aopTable[i].satDcsId,
                      (aopTable[i].downlinkStatus == SAT_DNLK_OFF) ? "SAT_DNLK_OFF" : (aopTable[i].downlinkStatus == SAT_DNLK_ON_WITH_A3) ? "SAT_DNLK_ON_WITH_A3" : "SAT_DNLK_ON_WITH_A4",
                      (aopTable[i].uplinkStatus == SAT_UPLK_OFF) ? "SAT_UPLK_OFF" : (aopTable[i].uplinkStatus == SAT_UPLK_ON_WITH_A2) ? "SAT_UPLK_ON_WITH_A2" : (aopTable[i].uplinkStatus == SAT_UPLK_ON_WITH_A3) ? "SAT_UPLK_ON_WITH_A3" : (aopTable[i].uplinkStatus == SAT_UPLK_ON_WITH_A4) ? "SAT_UPLK_ON_WITH_A4" : "SAT_UPLK_ON_WITH_NEO",
                      aopTable[i].bulletin.gpsYear, aopTable[i].bulletin.gpsMonth, aopTable[i].bulletin.gpsDay, aopTable[i].bulletin.gpsHour, aopTable[i].bulletin.gpsMinute, aopTable[i].bulletin.gpsSecond,
                      aopTable[i].semiMajorAxisKm, aopTable[i].inclinationDeg, aopTable[i].ascNodeLongitudeDeg,
                      aopTable[i].ascNodeDriftDeg, aopTable[i].orbitPeriodMin, aopTable[i].semiMajorAxisDriftMeterPerDay, aopTable[i].entryName);
    }
    SerialPrintDebugln("};");
}
