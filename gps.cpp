#include "gps.h"
#include "conf.h"
#include "logging.h"
#include "power_sleep.h"   // for ConnectPeripherals
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
#include <SD.h>

// Objects owned by the main sketch (popup-buoy.ino).
extern TinyGPSPlus gps;
extern SoftwareSerial gpsSerial;
extern RTC_DS3231 rtcExt;

// GPS track file, owned by this module (GPSfilename declared extern in gps.h).
const char *GPSfilename = "/GPS_track.csv";
File GpsTrackFile;

// GPS fix data owned by this module (declared extern in gps.h).
double gpsLat, gpsLong;
uint8_t gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond;
uint16_t gpsYear;
bool gpsFix;
uint32_t epochTime;
int maxGPSTimeout;  // GPS acquisition timeout (ms), loaded from conf.txt

// Last known fix, kept in the ESP32's RTC slow memory so it survives deep sleep and can be fed
// back to the receiver as a warm-start hint. It is lost on a full power cycle, in which case we
// simply skip the position hint. Deliberately not read back from GPS_track.csv: that file grows
// and would have to be scanned to its end on every wake-up.
RTC_DATA_ATTR static double lastFixLat = 0.0;
RTC_DATA_ATTR static double lastFixLon = 0.0;
RTC_DATA_ATTR static bool   lastFixValid = false;

// GPS time runs ahead of UTC by the accumulated leap seconds (18 s since Jan 2017).
// Only used to build a coarse time hint, and we declare +/-30 s accuracy, so a future
// leap second would not invalidate it.
static const uint32_t GPS_UTC_LEAP_SECONDS = 18;
static const uint32_t GPS_EPOCH_IN_UNIX    = 315964800UL;  // 1980-01-06 00:00:00 UTC

// Send a UBX frame, computing the 8-bit Fletcher checksum over class/id/length/payload.
// (The receiver silently discards any frame whose checksum or declared length is wrong.)
static void sendUBX(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t len) {
  uint8_t header[6] = { 0xB5, 0x62, msgClass, msgId, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ckA = 0, ckB = 0;
  for (uint8_t i = 2; i < 6; i++) { ckA += header[i]; ckB += ckA; }   // checksum skips the 2 sync bytes
  for (uint16_t i = 0; i < len; i++) { ckA += payload[i]; ckB += ckA; }

  gpsSerial.write(header, sizeof(header));
  gpsSerial.write(payload, len);
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
}

#ifdef SERIAL_DEBUG
// Debug aid: wait briefly for the receiver's UBX-ACK-ACK for a configuration message, so we can
// see in the log whether the frame was accepted (a malformed frame is silently discarded).
static bool waitForUbxAck(uint8_t msgClass, uint8_t msgId, uint16_t timeoutMs) {
  uint8_t expected[10] = { 0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, msgClass, msgId, 0, 0 };
  uint8_t ckA = 0, ckB = 0;
  for (uint8_t i = 2; i < 8; i++) { ckA += expected[i]; ckB += ckA; }
  expected[8] = ckA; expected[9] = ckB;

  uint8_t idx = 0;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (gpsSerial.available()) {
      uint8_t b = gpsSerial.read();
      if (b == expected[idx]) {
        if (++idx == sizeof(expected)) return true;   // complete ACK-ACK for this message
      } else {
        idx = (b == 0xB5) ? 1 : 0;                    // resync on a new frame start
      }
    }
  }
  return false;
}
#endif

// Feed the receiver a coarse time (from the DS3231) and our last known position, so it can
// warm-start instead of cold-starting after the relay has powered it down. Both hints are
// optional and are sent with deliberately loose accuracies: they only narrow the search, and
// if one turns out to be inconsistent the receiver just falls back to a normal acquisition.
static void sendGpsAiding() {
  uint8_t ini[48] = { 0 };   // UBX-AID-INI payload is exactly 48 bytes
  uint32_t flags = 0;

  // --- position hint: last fix, carried across deep sleep in RTC memory ---
  if (lastFixValid) {
    int32_t lat = (int32_t)(lastFixLat * 1e7);   // degrees * 1e-7
    int32_t lon = (int32_t)(lastFixLon * 1e7);
    uint32_t posAcc = 100000;                    // 1 km in cm: loose on purpose (the buoy drifts)
    memcpy(&ini[0],  &lat, 4);
    memcpy(&ini[4],  &lon, 4);
    memcpy(&ini[12], &posAcc, 4);                // ecefZOrAlt (offset 8) stays 0 and is flagged invalid
    flags |= 0x01;   // position valid
    flags |= 0x20;   // position given as lat/lon/alt
    flags |= 0x40;   // altitude invalid
  }

  // --- time hint: only when the RTC is trustworthy ---
  DateTime now = rtcExt.now();
  if (!rtcExt.lostPower() && now.year() >= 2025) {
    uint32_t gpsSeconds = (uint32_t)(now.unixtime() - GPS_EPOCH_IN_UNIX + GPS_UTC_LEAP_SECONDS);
    uint16_t week  = (uint16_t)(gpsSeconds / 604800UL);
    uint32_t towMs = (gpsSeconds % 604800UL) * 1000UL;
    uint32_t tAccMs = 30000;                     // +/-30 s: an honest, coarse hint
    memcpy(&ini[18], &week, 2);
    memcpy(&ini[20], &towMs, 4);
    memcpy(&ini[28], &tAccMs, 4);
    flags |= 0x02;   // time valid
  }

  if (flags == 0) return;                        // neither hint available -> nothing to send
  memcpy(&ini[44], &flags, 4);
  sendUBX(0x0B, 0x01, ini, sizeof(ini));         // class 0x0B (AID), id 0x01 (INI)

  #ifdef SERIAL_DEBUG
    SerialPrintDebugln("GPS aiding sent (flags 0x" + String(flags, HEX) + ")");
  #endif
}

void configGPS() {
  // UBX-CFG-NAV5: sea dynamic model + 2D-only fix at sea level.
  // A surface buoy never needs altitude, and a 2D fix only needs 3 satellites instead of 4,
  // so it locks faster and copes better with waves blocking part of the sky.
  uint8_t nav5[36] = { 0 };                 // payload must be exactly 36 bytes
  nav5[0]  = 0x05; nav5[1] = 0x00;          // mask: apply dynModel (bit0) + fixMode/fixedAlt (bit2)
  nav5[2]  = 5;                             // dynModel = 5 (Sea)
  nav5[3]  = 1;                             // fixMode  = 1 (2D only)
                                            // fixedAlt (offset 4..7) = 0 -> sea level
  nav5[8]  = 0x10; nav5[9] = 0x27;          // fixedAltVar = 10000 (1 m^2), u-blox default
  nav5[12] = 5;                             // minElev = 5 deg
  nav5[14] = 0xFA; nav5[15] = 0x00;         // pDop = 25.0
  nav5[16] = 0xFA; nav5[17] = 0x00;         // tDop = 25.0
  nav5[18] = 0x64; nav5[19] = 0x00;         // pAcc = 100 m
  nav5[20] = 0x2C; nav5[21] = 0x01;         // tAcc = 300 m
  nav5[23] = 60;                            // dgpsTimeOut, u-blox default

  sendUBX(0x06, 0x24, nav5, sizeof(nav5));  // class 0x06 (CFG), id 0x24 (NAV5)

  #ifdef SERIAL_DEBUG
    if (waitForUbxAck(0x06, 0x24, 300)) {
      SerialPrintDebugln("GPS CFG-NAV5 acknowledged (Sea model, 2D fix)");
    } else {
      SerialPrintDebugln("WARNING: no ACK for GPS CFG-NAV5");
    }
  #endif

  sendGpsAiding();   // warm-start hints (AID-INI is not acknowledged by the receiver)

  delay(500);  // Espera para permitir que el GPS procese la configuración
}

bool gpsAcquireSatellites() {
  ConnectPeripherals(true, GPS_KIM);
  gpsSerial.begin(GPSBaud);
  gps = TinyGPSPlus();  // Reset the GPS
  delay(10);
  configGPS();
  unsigned long startTime = millis();  // Marca el tiempo de inicio
  SerialPrintDebug("GPS acquiring data------>");
  while (millis() - startTime < 30000 && (digitalRead(PB_1) == true)) {  // Tiempo límite de 30 segundos
    if (gpsSerial.available() > 0 ) {
      char c = gpsSerial.read();
      //SerialPrintDebug(String(c));
      gps.encode(c);  // Decodificar los datos del GPS
      if (gps.satellites.isValid() && gps.satellites.value() > 0) {  // Comprobar si hay satélites detectados
        ConnectPeripherals(false, GPS_KIM);
        return true;  // Retorna true si encuentra al menos un satélite
      }
      //delay(10);
    }
  }
  ConnectPeripherals(false, GPS_KIM);
  return false;  // Retorna false si no se detectan satélites en 30 segundos
}

void gpsAcquireData(double &gpsLat, double &gpsLong, uint16_t &gpsYear, uint8_t &gpsMonth, uint8_t &gpsDay, uint8_t &gpsHour, uint8_t &gpsMinute, uint8_t &gpsSecond, uint32_t &epochTime, bool &gpsFix) {

  gps = TinyGPSPlus();  // Reset the GPS
  delay(10);
  configGPS();
  int gpsState = 0;
  int initialTime = millis();
  gpsFix = false;

  while (gpsState == 0 && millis() < (maxGPSTimeout + initialTime) && digitalRead(PB_1) == true) {
    #ifdef TEST_FORCE_GPS_VILANOVA_PB2
      // === TEST AID (permanent) === press PB_2 to inject a GPS fix indoors so the Surface/SPP path can run.
      // Uses Vilanova i la Geltru coordinates and takes the time from the RTC. Harmless in the field:
      // it only triggers on a deliberate button press. Toggle with TEST_FORCE_GPS_VILANOVA_PB2 in conf.h.
      if (digitalRead(PB_2) == false) {
        gpsLat  = 41.2241;   // Vilanova i la Geltru latitude (N)
        gpsLong = 1.7260;    // Vilanova i la Geltru longitude (E)
        DateTime nowRTC = rtcExt.now();
        gpsYear   = nowRTC.year();
        gpsMonth  = nowRTC.month();
        gpsDay    = nowRTC.day();
        gpsHour   = nowRTC.hour();
        gpsMinute = nowRTC.minute();
        gpsSecond = nowRTC.second();
        epochTime = nowRTC.unixtime();
        gpsFix    = true;
        gpsState  = 1;
        SerialPrintDebugln(F("[TEST] PB_2 pressed -> forcing Vilanova i la Geltru GPS fix from RTC"));
        break;
      }
    #endif
    //SerialPrintDebugln("GPS acquiring data------>");
    while (gpsSerial.available() > 0 && millis() < (maxGPSTimeout + initialTime) && digitalRead(PB_1) == true) {
      if (gps.encode(gpsSerial.read())) {

        SerialPrintDebug(F("Location: "));
        if (gps.location.isValid()) {
          gpsLat = gps.location.lat();
          gpsLong = gps.location.lng();
          gpsState = 1;
          SerialPrintDebug(String(gpsLat,6));
          SerialPrintDebug(F(";"));
          SerialPrintDebug(String(gpsLong,6));
          gpsFix = true;
          lastFixLat = gpsLat;      // remember the real fix for the next warm start
          lastFixLon = gpsLong;
          lastFixValid = true;
        } else {
          //SerialPrintDebug(F("INVALID"));
          gpsState = 0;
        }

        SerialPrintDebug(F("  Date/Time: "));
        if (gps.date.isValid()) {
          gpsYear = gps.date.year();
          gpsMonth = gps.date.month();
          gpsDay = gps.date.day();
          SerialPrintDebug(gpsDay);
          SerialPrintDebug(F("/"));
          SerialPrintDebug(gpsMonth);
          SerialPrintDebug(F("/"));
          SerialPrintDebug(gpsYear);
        } else {
          SerialPrintDebug(F("INVALID"));
          gpsState = 0;
        }

        SerialPrintDebug(F("  "));
        if (gps.time.isValid()) {
          gpsHour = gps.time.hour();
          gpsMinute = gps.time.minute();
          gpsSecond = gps.time.second();
          if (gpsHour < 10) SerialPrintDebug(F("0"));
          SerialPrintDebug(gpsHour);
          SerialPrintDebug(F(":"));
          if (gpsMinute < 10) SerialPrintDebug(F("0"));
          SerialPrintDebug(gpsMinute);
          SerialPrintDebug(F(":"));
          if (gpsSecond < 10) SerialPrintDebug(F("0"));
          SerialPrintDebug(gpsSecond);
          SerialPrintDebugln(" ");
        } else {
          SerialPrintDebugln(F("INVALID"));
          gpsState = 0;
        }

        if (gps.time.isValid() && gps.date.isValid() && gps.location.isValid()) {
          DateTime now2 = DateTime(gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond);  //for the kineisMessage we use the gps epoch time
          SerialPrintDebug("Epoch time: ");
          epochTime = now2.unixtime();
          SerialPrintDebugln(epochTime);
        }
        //delay(100);
      }
    }
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      SerialPrintDebugln(F("No GPS detected: check wiring."));
    }
    //gpsSerial.end();
    delay(300);  // delay loop while
  }

  if (gpsFix) {
    SerialPrintDebugln("GPS acquiring data------>DONE");
  } else {
    SerialPrintDebugln("GPS acquiring data------TIMEOUT");
  }
}

void gpsSave(double &gpsLat, double &gpsLong, uint16_t &gpsYear, uint8_t &gpsMonth, uint8_t &gpsDay, uint8_t &gpsHour, uint8_t &gpsMinute, uint8_t &gpsSecond, uint32_t &epochTime, bool &gpsFix){
  if (gpsFix) {  // If the GPS has been set
    //save GPS location to track file
    if (saveGPStoSD(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)) {
      writeLogFile("Stored GPS data to SD succesfully");
    } else {
      writeLogFile("Failed to store GPS data to SD ");
      }
  } else {
    writeLogFile("Failed to fix GPS");
    // When GPS fails to fix in phase 4, the GPS_track.csv is not updated but it is set  gpsLong = FFFFFFFF and gpsLat = FFFFFFFF with GPSfix=false. These insfo is transmitted for ARGOS dopler detection
    gpsLat = 200;
    gpsLong = 200;
    //get the epoch time from the RTC
    DateTime now = rtcExt.now();
    epochTime = now.unixtime();
  }
}

bool saveGPStoSD(double &gpsLat, double &gpsLong, uint16_t &gpsYear, uint8_t &gpsMonth, uint8_t &gpsDay, uint8_t &gpsHour, uint8_t &gpsMinute, uint8_t &gpsSecond) {
  //*******Missing to add header when the file is create

  SerialPrintDebugln("Saving GPS data to SD ---");
  //Go to root directory

  //Open file and create if it doeesn't exist
  GpsTrackFile = SD.open(GPSfilename, FILE_APPEND);  //GPSfilename is the file name to be created and FILE_WRITE is a command to create file.
  if (!GpsTrackFile) {
    // the file has been opened correctly
    SerialPrintDebug(GPSfilename);
    SerialPrintDebugln("  has NOT been opened correctly");
    GpsTrackFile.close();  //Closing the file
    return false;
  }

  //append data loop
  if (!GpsTrackFile.print(gpsLat, 6)) {
    return false;
  }

  if (!GpsTrackFile.print(",")) {
    return false;
  }

  if (!GpsTrackFile.print(gpsLong, 6)) {
    return false;
  }

  if (!GpsTrackFile.print(",")) {
    return false;
  }
  // In the trackFile we save LAt Long + GPS time
  if (!GpsTrackFile.print(gpsYear)) {
    return false;
  }

  if (!GpsTrackFile.print(",")) {
    return false;
  }

  if (!GpsTrackFile.print(gpsMonth)) {
    return false;
  }

  if (!GpsTrackFile.print(",")) {
    return false;
  }

  if (!GpsTrackFile.print(gpsDay)) {
    return false;
  }

  if (!GpsTrackFile.print(",")) {
    return false;
  }

  if (!GpsTrackFile.print(gpsHour)) {
    return false;
  }

  if (!GpsTrackFile.print(",")) {
    return false;
  }

  if (!GpsTrackFile.print(gpsMinute)) {
    return false;
  }

  if (!GpsTrackFile.print(",")) {
    return false;
  }

  if (!GpsTrackFile.print(gpsSecond)) {
    return false;
  }

  if (!GpsTrackFile.println(";")) {
    return false;
  }


  //close file
  GpsTrackFile.close();  //Closing the file
  return true;
}
