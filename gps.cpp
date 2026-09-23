#include "gps.h"
#include "conf.h"
#include "logging.h"
#include "power_sleep.h"   // for ConnectPeripherals
#include "board.h"
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

void gpsPowerOn() {
  if (!boardIsV2()) return;
  // Port first, supply second. The NEO-M8N switches its own UART receiver off
  // after framing errors, and a floating GPIO2 while its rail comes up produces
  // exactly that - measured on the V2 bench, 22 Sep 2026. Opening the port
  // first holds the line at its idle high before the module wakes.
  gpsSerialBegin();
  digitalWrite(GPS_EN_V2, HIGH);
  SerialPrintDebugln("GPS powered (GPIO" + String(GPS_EN_V2) + ")");
}

void gpsPowerOff() {
  if (!boardIsV2()) return;
  // Port first again, and the line left low: an idle-high GPIO2 would feed the
  // unpowered module through its ESD clamp.
  gpsSerial.end();
  pinMode(TXPin_GPS, OUTPUT);
  digitalWrite(TXPin_GPS, LOW);
  digitalWrite(GPS_EN_V2, LOW);
  SerialPrintDebugln("GPS unpowered");
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
#endif  // GPS_DEBUG_NMEA_GSA

// Send a UBX frame, computing the 8-bit Fletcher checksum over class/id/length/payload.
static void sendUBX(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t len) {
  uint8_t header[6] = { 0xB5, 0x62, msgClass, msgId, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ckA = 0, ckB = 0;
  for (uint8_t i = 2; i < 6; i++) { ckA += header[i]; ckB += ckA; }   // checksum skips the 2 sync bytes
  for (uint16_t i = 0; i < len; i++) { ckA += payload[i]; ckB += ckA; }

  gpsSerial.write(header, sizeof(header));
  if (len) gpsSerial.write(payload, len);
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
  gpsSerial.flush();
}

// Waits for a UBX frame of the given class/id and copies its payload. Returns
// the payload length, or -1 on timeout. NMEA keeps flowing meanwhile, so the
// sync bytes are hunted rather than expected.
static int waitUBX(uint8_t cls, uint8_t id, uint8_t *out, size_t outSize, uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  int state = 0;
  uint8_t gotCls = 0, gotId = 0;
  uint16_t len = 0, idx = 0;

  while (millis() - t0 < timeoutMs) {
    while (gpsSerial.available()) {
      const uint8_t c = gpsSerial.read();
      switch (state) {
        case 0: state = (c == 0xB5) ? 1 : 0; break;
        case 1: state = (c == 0x62) ? 2 : 0; break;
        case 2: gotCls = c; state = 3; break;
        case 3: gotId = c;  state = 4; break;
        case 4: len = c;    state = 5; break;
        case 5: len |= (uint16_t)c << 8; idx = 0; state = (len == 0) ? 7 : 6; break;
        case 6: if (idx < outSize) out[idx] = c;
                if (++idx >= len) state = 7;
                break;
        case 7: state = 8; break;            // checksum A, not verified
        case 8:
          if (gotCls == cls && gotId == id) return (int)len;
          state = 0;
          break;
      }
    }
    delay(2);
  }
  return -1;
}

// ACK-ACK (true) or ACK-NAK / nothing (false) for the message cls/id just sent.
static bool waitAck(uint8_t cls, uint8_t id) {
  const uint32_t t0 = millis();
  uint8_t p[2];
  while (millis() - t0 < UBX_ACK_TIMEOUT_MS) {
    // Only ACK-ACK (0x05 0x01) naming this message counts. A NAK (0x05 0x00)
    // is skipped by the hunt and simply ends in the timeout.
    const uint32_t left = UBX_ACK_TIMEOUT_MS - (millis() - t0);
    int n = waitUBX(0x05, 0x01, p, sizeof(p), left);
    if (n == 2 && p[0] == cls && p[1] == id) return true;
    if (n < 0) return false;
  }
  return false;
}

// CFG-NAVX5 offsets on the u-blox 8 (payload version 2): mask1 bit 14 tells the
// receiver to apply the AOP settings, aopCfg bit 0 is useAOP.
#define UBX_NAVX5_AOPCFG_OFFSET 27
#define UBX_NAVX5_MASK1_AOP     0x4000

// V2 only. Both settings live in the receiver's RAM and the GPS is powered off
// after every fix, so they are sent at every power-up - never saved to its
// flash, the same rule as for the satellite modules.
static void configM8N() {
  // Let the module boot: the first NMEA character says it is up.
  const uint32_t t0 = millis();
  while (millis() - t0 < UBX_BOOT_TIMEOUT_MS && !gpsSerial.available()) delay(5);
  while (gpsSerial.available()) gpsSerial.read();

  // Sea dynamic model: altitude held at sea level, little vertical motion, low
  // speeds. A tighter model for the filter than the default "portable", which
  // also keeps waves from being read as altitude. Only dynModel is masked in,
  // so the fix mode stays at the receiver's default (auto 2D/3D). Measured on
  // the bench 22 Sep 2026: ACK, and dynModel=5 read back.
  uint8_t nav5[36] = { 0 };
  nav5[0] = 0x01; nav5[1] = 0x00;          // mask: dyn only
  nav5[2] = 5;                             // dynModel 5 = Sea
  sendUBX(0x06, 0x24, nav5, sizeof(nav5));
  const bool navOk = waitAck(0x06, 0x24);

  // AssistNow Autonomous: the receiver predicts its own ephemeris from what it
  // has already received, which makes a start after more than the ~2 h
  // ephemeris validity faster. Read-modify-write, so no other NAVX5 field is
  // disturbed. Enables cleanly (aopCfg read back 0x01, 22 Sep); whether it ever
  // gets to compute with the GPS on for seconds at a time is to be seen in the
  // field - a fix of seconds after a gap of hours will say so.
  bool aopOk = false;
  uint8_t navx5[64];
  sendUBX(0x06, 0x23, nullptr, 0);
  const int n = waitUBX(0x06, 0x23, navx5, sizeof(navx5), UBX_ACK_TIMEOUT_MS);
  if (n > UBX_NAVX5_AOPCFG_OFFSET && n <= (int)sizeof(navx5)) {
    navx5[2] |= UBX_NAVX5_MASK1_AOP & 0xFF;
    navx5[3] |= (UBX_NAVX5_MASK1_AOP >> 8) & 0xFF;
    navx5[UBX_NAVX5_AOPCFG_OFFSET] |= 0x01;
    sendUBX(0x06, 0x23, navx5, (uint16_t)n);
    aopOk = waitAck(0x06, 0x23);
  }

  SerialPrintDebugln(String("GPS config: sea model ") + (navOk ? "ACK" : "FAILED")
                     + ", AssistNow Autonomous " + (aopOk ? "ACK" : "FAILED"));
  // Logged only when something did not take, so a healthy buoy adds nothing.
  // A failure here with NMEA still flowing is the receiver having shut its
  // UART input down - see gpsPowerOn().
  if (!navOk || !aopOk) {
    writeLogFile(String("GPS config not accepted: sea model ") + (navOk ? "ok" : "FAILED")
                 + ", AssistNow Autonomous " + (aopOk ? "ok" : "FAILED"));
  }
}

void configGPS() {
  if (boardIsV2()) {
    configM8N();
    return;
  }
  // V1: this receiver is NOT a u-blox: it reports $GNGSA with an NMEA 4.1 systemId field (GPS + BeiDou)
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
  if (boardIsV2()) {
    gpsPowerOn();
  } else {
    ConnectPeripherals(true, GPS_KIM);
    gpsSerialBegin();
  }
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
        if (boardIsV2()) gpsPowerOff(); else ConnectPeripherals(false, GPS_KIM);
        return true;  // Retorna true si encuentra al menos un satélite
      }
      //delay(10);
    }
  }
  if (boardIsV2()) gpsPowerOff(); else ConnectPeripherals(false, GPS_KIM);
  return false;  // Retorna false si no se detectan satélites en 30 segundos
}

void gpsAcquireData(double &gpsLat, double &gpsLong, uint16_t &gpsYear, uint8_t &gpsMonth, uint8_t &gpsDay, uint8_t &gpsHour, uint8_t &gpsMinute, uint8_t &gpsSecond, uint32_t &epochTime, bool &gpsFix) {

  // V2: the GPS is powered for the fix only, in every state, FRM included - it
  // has its own switch and nothing else needs it on. The coin cell keeps its
  // backup memory, so the next fix is a warm start of seconds (3.65 s measured
  // after a 15 s cut). No-op on a V1.
  gpsPowerOn();

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

  gpsPowerOff();

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
