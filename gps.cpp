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

void configGPS() {
  // Comando UBX CFG-NAV5 para configurar 2D fix
  uint8_t ubxConfig[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, // Cabecera
    0x01, 0x00, // Mask: Apply dynamic model and fix mode
    0x03, 0x00, // Dynamic model: Airborne <1g (se puede cambiar según el uso)
    0x01, 0x00, // Fix mode: 0x01, 0x00, = 2D only, 0x02,  <-- Cambio aquí
    0x00, 0x00, 0x00, 0x00, // Fixed alt
    0x00, 0x00, 0x00, 0x00, // Fixed alt var
    0x00, 0x00, 0x00, 0x00, // Min elev, drLimit, pDop, tDop
    0x00, 0x00, 0x00, 0x00, // pAcc, tAcc, static hold threshold
    0x00, 0x00, 0x00, 0x00, // Reserved
    0x00, 0x00, 0x00, 0x00, // Reserved
    0x00, 0x00, 0x00, 0x00  // Reserved
  };

  // Enviar el comando UBX al GPS para configurar el modo 2D
  for (int i = 0; i < sizeof(ubxConfig); i++) {
    gpsSerial.write(ubxConfig[i]);
  }

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
