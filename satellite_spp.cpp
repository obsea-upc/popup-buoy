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
    3.0f,                                                             //< Minimum duration (default 5 minutes)
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
    // Measured on hardware: 10 s and 5 s agree exactly, 1 s confirms within
    // 4 s and 1 deg, and the cost is 26 ms per satellite instead of 9.
    10                                                                //< Computation step
  };

  struct SatelliteNextPassPrediction_t nextPass;
  struct SatelliteNextPassPrediction_t earliestPass;
  bool passFound = false;
  uint8_t earliestIdx = 0;   // which AOP entry produced earliestPass, for the name

  for (uint8_t i = 0; i < nbSatsInAopTable; i++) {
    // PREVIPASS returns false when it finds no pass for this satellite, and that
    // return value used to be discarded. With "i == 0" accepting the first entry
    // unconditionally, a satellite with no pass left earliestPass holding epoch 0
    // - and since nothing is earlier than zero, no later satellite could replace
    // it. That is the "XX on 7/1/2004, -712731974 sec" in the 9 Aug logs, and it
    // is what kept the retry loop spinning for five hours.
    if (!PREVIPASS_compute_next_pass(&prepasConfiguration, &aopTable[i], 1, &nextPass)) {
      continue;
    }
    // A pass that is still in progress is worth returning: the caller can reuse
    // what is left of it (the overlap branch). One that has already ENDED is not
    // - it can only come back as a negative time and be thrown away as an error.
    // epoch is a Unix timestamp here, same base as the RTC (see the display code
    // below, which converts it with EPOCH_90_TO_70_OFFSET).
    if (nextPass.epoch + (uint32_t)nextPass.duration <= now.unixtime()) {
      continue;
    }
    if (!passFound || nextPass.epoch < earliestPass.epoch) {
      earliestPass = nextPass;
      earliestIdx = i;
      passFound = true;
      delay(100);
    }
  }

  if (!passFound) {
    // Nothing above MinElev in the whole prediction window. Report it as a
    // negative time with zero coverage so the caller takes its error path, which
    // is now bounded, instead of us inventing a schedule out of an empty result.
    writeLogFile("SPP: no satellite pass found above " + String(MinElev) + " deg");
    SetCoverageDurationTo_0();
    return -1;
  }
  //messageLogFile = "For the SPP : Next satellite epoch : " + String(earliestPass.epoch) + " and epoch now : " + String(now.unixtime());
  //writeLogFile(messageLogFile);

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

  PREVIPASS_UTIL_date_stu90_calendar(earliestPass.epoch - EPOCH_90_TO_70_OFFSET,
                                     &viewable_timedata);




  String response = "Data: " + String(viewable_timedata.gpsDay) + "/" + String(viewable_timedata.gpsMonth) + "/" + String(viewable_timedata.gpsYear) + ".  The next satellite will be "
  + String(satNameTwoChars) +     " at " + String(viewable_timedata.gpsHour) + ":" + String(viewable_timedata.gpsMinute) + ":" + String(viewable_timedata.gpsSecond) + "UTC, with a duration of "
  + String(int(earliestPass.duration) / 60) + " min and "  + String(int(earliestPass.duration) % 60) + " sec and a maximum elevation of " + String(int(earliestPass.elevationMax)) + "º";
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
  Decimal_CoverageDuration = int(earliestPass.duration);
  eepromSaveTimeCoverage(Decimal_CoverageDuration);
  Decimal_CoverageDuration = eepromReadCoverageDuration();
  SerialPrintDebugln(" Time of coverage next satellite : " + String(Decimal_CoverageDuration));

  messageLogFile = "Next Satelitte : Time before next satellite :" + String(diff) + " sec and coverage : " + String(Decimal_CoverageDuration) + String(" sec");
  writeLogFile(messageLogFile);

  return diff;
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
