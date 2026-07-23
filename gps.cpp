#include "gps.h"
#include "conf.h"
#include "logging.h"
#include "power_sleep.h"   // for ConnectPeripherals
#include <TinyGPSPlus.h>
#include <RTClib.h>
#include <SD.h>

// Objects owned by the main sketch (popup-buoy.ino).
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;
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

// (lastFix* is still updated on every real fix: it costs nothing and is what we would feed back to
//  the receiver as a warm-start hint once we know which protocol this module accepts.)

#ifdef GPS_DEBUG_NMEA_GSA
// TEMPORARY diagnostic: echo the raw GSA sentences so the fix mode can be read directly from
// what the receiver actually sends (GSA field 2: 1 = no fix, 2 = 2D, 3 = 3D). Also tells us
// whether the module emits GSA at all. Remove the #define in conf.h to switch it off.
static void debugEchoGsa(char c) {
  static char line[96];
  static uint8_t n = 0;
  if (c == '\n' || c == '\r') {
    if (n > 6) {
      line[n] = '\0';
      if (strstr(line, "GSA")  != NULL ||
          strstr(line, "PCAS") != NULL ||
          strstr(line, "TXT")  != NULL) SerialPrintDebugln(String("NMEA> ") + line);
    }
    n = 0;
  } else if (n < sizeof(line) - 1) {
    line[n++] = c;
  }
}
#endif

// Fix mode reported by the receiver in the NMEA GSA sentence: 1 = no fix, 2 = 2D, 3 = 3D.
// This is the ground truth for whether the 2D configuration actually took effect.
static TinyGPSCustom gsaFixMode;

// Open the GPS port on hardware UART1, remapped to the GPS pins through the GPIO matrix
// (UART0 is the USB debug console and UART2 drives the KIM module).
// A real UART instead of SoftwareSerial: same reception, but more robust, no bit-banging on the
// CPU and ~6 kB less flash.
//
// The receiver ignores every configuration command we send (UBX and CASIC $PCAS alike). This was
// traced on a bare ESP32 + GPS test rig: a GPIO17->GPIO16 loopback proved the UART transmit path
// is perfect (ASCII and binary UBX frames both came back byte for byte), so the module is
// receiving the commands and discarding them. These "NEO-6M" modules are clones -- they emit
// $GNRMC/$GNGSA with an NMEA 4.1 BeiDou systemId, which a genuine GPS-only NEO-6M never could --
// and they implement no configuration protocol. Configuring the receiver would need a genuine
// u-blox module; it is not a wiring or pin problem.
void gpsSerialBegin() {
  gpsSerial.setRxBufferSize(512);   // the receiver streams NMEA continuously; never starve it
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin_GPS, TXPin_GPS);
}

#ifdef GPS_DEBUG_NMEA_GSA
// Send an NMEA sentence with its checksum: sendNmea("PCAS06,0") -> "$PCAS06,0*1B\r\n".
// Cheap "NEO-6M" modules are often CASIC/AT6558 based and are configured with these proprietary
// $PCAS sentences rather than with u-blox UBX frames.
static void sendNmea(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  char sentence[64];
  snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, cs);
  gpsSerial.print(sentence);
}

// Send a UBX frame, computing the 8-bit Fletcher checksum over class/id/length/payload.
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
#endif  // GPS_DEBUG_NMEA_GSA

void configGPS() {
  // This receiver is NOT a u-blox: it reports $GNGSA with an NMEA 4.1 systemId field (GPS + BeiDou)
  // and never acknowledged UBX, so the previous UBX-CFG-NAV5 / AID-INI path was dead weight (it also
  // cost ~0.9 s per acquisition in ACK retries) and has been removed.
  // Cheap multi-GNSS modules sold as "NEO-6M" are usually CASIC/AT6558 based; probe for that with a
  // $PCAS version query so we know which protocol we can actually configure it with.
  #ifdef GPS_DEBUG_NMEA_GSA
    while (gpsSerial.available()) gpsSerial.read();   // clear the backlog so the reply is easy to spot
    sendNmea("PCAS06,0");                             // CASIC product/version query

    // TX PATH TEST: ask the receiver to stop emitting GSA (UBX-CFG-MSG, NMEA class 0xF0, GSA id 0x02,
    // rate 0). This only touches GSA -- GGA/RMC, which TinyGPS needs for the fix, are left alone, and
    // the change lives in RAM only, so a power cycle restores it.
    // If the GSA lines below stop appearing, our transmit path works (and the module is u-blox).
    // If they keep coming, nothing we send is reaching the module.
    uint8_t cfgMsgGsaOff[3] = { 0xF0, 0x02, 0x00 };
    sendUBX(0x06, 0x01, cfgMsgGsaOff, sizeof(cfgMsgGsaOff));
    SerialPrintDebugln("GPS TX TEST: sent $PCAS06 probe + UBX-CFG-MSG to silence GSA");
  #endif

  delay(200);  // let the receiver settle before we start parsing
}

bool gpsAcquireSatellites() {
  ConnectPeripherals(true, GPS_KIM);
  gpsSerialBegin();
  gps = TinyGPSPlus();  // Reset the GPS
  gsaFixMode.begin(gps, "GNGSA", 2);   // re-register after the reset. This module emits GNGSA (multi-GNSS), not GPGSA
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
  gsaFixMode.begin(gps, "GNGSA", 2);   // re-register after the reset. This module emits GNGSA (multi-GNSS), not GPGSA
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
      char nmeaChar = gpsSerial.read();
      #ifdef GPS_DEBUG_NMEA_GSA
        debugEchoGsa(nmeaChar);
      #endif
      if (gps.encode(nmeaChar)) {

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
    // Ground truth for the 2D configuration: 2 = 2D fix, 3 = 3D fix.
    if (gsaFixMode.isValid()) {
      writeLogFile("GPS fix mode (NMEA GSA): " + String(gsaFixMode.value()) + "  (2 = 2D, 3 = 3D)");
    }
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
