/*******************************************************************************
*   00_MasterPoopUpBuoy.ino
*
*   This file contains the master-code for the PopUpBuoys....
*
*   HARDWARE:
*   - ESP32-Wroom-32 (EZSBC).
*   - MicroSD Card Adapter
*   - x3 leds
*   - x3 push button
*   - State Solid Relay CN024D05 (crydom) x2
*   - GPS module (GY-NEO 6MV2)
*   - Kineis (KIM1: version V2)
*   - RTC module
*   - Step-UP converter (U3V16F5)
*
*    !!! IMPORTANT - Modify the secrets.h file for this project with your network connection and ThingSpeak channel details !!!

*
*   Matias Carandell (UPC)


******************************************************************************/
//IMPORTANT -- You must install the ESP32 board by Espressif Systems. Version 2.0.14! No newer!

#include "conf.h"
#include "secrets.h"
#include "logging.h"
#include "eeprom_store.h"
#include "gps.h"
#include "power_sleep.h"
#include "wifi_http.h"
#include "adc.h"
#include "ftp_download.h"
#include "usv_upload.h"
#include "satellite_spp.h"
#include "satellite_tx.h"
#include "config.h"
#include "Arduino.h"
#include "SD.h"
#include <RTClib.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "ESP32_FTPClient.h"
#include <TinyGPSPlus.h>
#include "KIM.h"
#include "ARRIBADA.h"
#include "sat_module.h"
#include <EEPROM.h>
#include <Wire.h>


//------ Configuration FTP server -------------------------------------------------------------------------------------
  ESP32_FTPClient ftp(SECRET_FTP_SERVER_IP, SECRET_FTP_SERVER_USER, SECRET_FTP_SERVER_PASS);

//------ Configuration for GPS module ---------------------------------------------------------------------------------
  TinyGPSPlus gps;                                 // The TinyGPSPlus object
  HardwareSerial gpsSerial(1);                     // UART1 (UART0 = USB debug, UART2 = KIM); pins set in gpsSerialBegin()

//------ Configuration for the satellite transmitter -------------------------------------------------------------------
// The KIM1 shield and the Arribada Argos SMD wing share one socket, one UART and one enable GPIO,
// and are never fitted at the same time. Both drivers are built on the same serial port; sat_module.*
// probes at boot and routes every call to whichever one actually answered.
  HardwareSerial kimSerial(2);  // hard coded no library
  KIM KIM(&kimSerial);          //with library
  ARRIBADA Arribada(&kimSerial);

//------ Kineis TX params (PWR2/PWR3/AFMT/delayKIM, kineisMessage/kineisdataMessage, new_line) now owned by satellite_tx.cpp

//------ Define Kineis SPP Parameters ---------------------------------------------------------------------------------
  int secondsBeforeNextStatellite;
  int Decimal_CoverageDuration;
  int CoverageState;  // Variable used to stock in EEPROM the fact that there will be or not coverage when the buoy wakes up
  int Counter_FailGPS;
  int Counter_FailWIFI;
  String messageLogFile = "";  // Variable used to write in the LogFile
  int timeSending;
  int fileSendingTime;
  int waitSendingTime;
  String number = "";
  float MinElev;
  float Bat_critlevel;

//------ GPS Acquiring Parameters now owned by gps.cpp (declared in gps.h) ---------------------------------------------

//------ Kineis data-progress (NbrMsgToSend, RowProgress, nbrSendingProgress, MaxRowDataFile, MaxNbrMsgSendingDataFile) now owned by satellite_tx.cpp

//------ Basic definitions ---------------------------------------------------------------------------------------------
  int currentState = INITIAL_STATE;  // current state of pop-up-buoy (0 submerged, 1 surfacing,...)
  int PBState = 0;                   // state of push buttons
  int idBuoy;
  // ActionType and ReleaseMode enums now defined in wifi_http.h
  int releaseFlag;
  ReleaseMode releaseMode;
  int sleeptime_h;
  int sleeptime_m;
//------ Timing config (sleeptime_s1_*, sleeptime_errorGPS_*, max_sleep_time_s, timetransm_GPS_*, maxFRM, maxWIFITimeout, sleepTimeWifiAttempt) now owned by config.cpp
  // year_lander..second_lander now owned by wifi_http.cpp
  int syncTime;
  int fileBlinkLed = 0;

//------ Definitions for time external RTC + ntp server  ---------------------------------------------------------------
  RTC_DS3231 rtcExt;

//------ Configuration of the NTP server -------------------------------------------------------------------------------
  const char *ntpServer = "pool.ntp.org";
  const long gmtOffset_sec = 0;      // UTC hour (in seconds)
  const int daylightOffset_sec = 0;  // Summer time change (in seconds)
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);

//------ SD files ------------------------------------------------------------------------------------------------------
  // SD file names/handles now owned by their modules: GPSfilename/GpsTrackFile (gps),
  // Log_filename/LogFile (logging), SD_data_filename/datamsgSD + SD_progress_filename (satellite_tx),
  // AOPfilename/AOPFile (satellite_spp), SD_config_filename/ConfigFileSD (config).
  File progressDataFileSD;   // shared: written by createProgressFile() here and by satellite_tx


//------ ADC ----------------------------------------------------------------------------------------------------------
  // ADC resolution, ADCreadHex and Vin_ADC now owned by adc.cpp

//-------SETUP FUNTION -----------------------------------------------------------------------------------
void setup() {

  //------- SERIAL SETUP -----------------------------------------------------------------------------------
    #ifdef SERIAL_DEBUG
      Serial.begin(SERIAL_DEBUG_BAUDRATE);
      SerialPrintDebugln("\n -------------WELCOME TO THE POP-UP-BUOY MASTER- " + String(SOFT_VERSION) + " " + String(COMPILE_DATE)  + "-------------\n\n");
    #endif

  //------- EEPROM DEFINITION ------------------------------------------------------------------------------
    EEPROM.begin(EEPROM_SIZE);
    initializeEEPROM();
    currentState = eepromReadState();
    SerialPrintDebug("CurrentState of POP_UP_BUOY: ");
    SerialPrintDebugln(stateName(currentState));

  //------- PB DEFINITION ----------------------------------------------------------------------------------
    pinMode(PB_1, INPUT_PULLUP);
    pinMode(PB_2, INPUT_PULLUP);
    pinMode(PB_3, INPUT_PULLUP);
    //pinMode(PB_3, INPUT); --> cambio

  //------- LED DEFINITION ---------------------------------------------------------------------------------
    pinMode(LED_R, OUTPUT);
    pinMode(LED_Y, OUTPUT);
    pinMode(LED_G, OUTPUT);

    digitalWrite(LED_Y, HIGH);  // turn on led to know that the board is in setup mode
    digitalWrite(LED_R, LOW);   // initialise off
    digitalWrite(LED_G, HIGH);  // initialise off

  //------- POWER RELAY DEFINITION -------------------------------------------------------------------------
    pinMode(GPS_KIM, OUTPUT); //future just KIM
    pinMode(SD_card, OUTPUT);
    //pinMode(GPS, OUTPUT)
    //digitalWrite(GPS_KIM, LOW); --> fixar-los a low d'inici
    //digitalWrite(GPS, LOW);
    digitalWrite(SD_card, HIGH);
    if (currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      digitalWrite(GPS_KIM, HIGH);
    } else{
      digitalWrite(GPS_KIM, LOW);
    }

  //------- EXTERNAL RTC SETUP---------------------------------------------------------------------------
    if (currentState == ST_CONFIG or currentState == ST_DEPLOY or currentState == ST_SEABED or currentState == ST_RELEASE or currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      Wire.begin();  // initialise I2C bus -> 100000 Hz (sda pin 21, scl pin 22)

      // initializing the rtc
      if (!rtcExt.begin()) {
        SerialPrintDebugln("Couldn't find RTC!");
        #ifdef SERIAL_DEBUG
          Serial.flush();
        #endif
        while (1) delay(10);
      }

      if (rtcExt.lostPower()) {
        // The oscillator stopped at some point (OSF set), so the kept time may be off. We deliberately
        // do NOT write the compile time here: __DATE__/__TIME__ is the moment the firmware was built,
        // which can be days or weeks stale, and adjust() would also clear OSF, making that stale value
        // look valid and then get propagated. Keep whatever the RTC still holds -- the last NTP/lander
        // sync -- and let the next NTP (CONFIG) or lander (SEABED) sync correct it. Only flag it here.
        // OSF stays set on purpose, as a record, until a real sync clears it.
        SerialPrintDebugln("WARNING: RTC reports lost power (OSF). Keeping last synced time; NOT writing compile time.");
      }
      //we don't need the 32K Pin, so disable it
      rtcExt.disable32K();

      // DS3231 INT/SQW pin (GPIO34) is used only as the ext0 deep-sleep wake source (see goToSleepRTC_*).
      // No runtime interrupt is attached: ext0 wakes the chip and it reboots; it never calls back into code.
      pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);

      // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
      // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
      rtcExt.clearAlarm(1);
      rtcExt.clearAlarm(2);

      // turn off alarm 2 (in case it isn't off already)
      // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
      rtcExt.disableAlarm(2);

      // stop oscillating signals at SQW Pin  otherwise setAlarm1 will fail
      rtcExt.writeSqwPinMode(DS3231_OFF);
    }
  //------- WIFI (CONFIG) CONNECTION SETUP ---------------------------------------------------------------------------
    if (currentState == ST_CONFIG) {
      SerialPrintDebugln("RTC Configuration. Push button 1 for NTP sync. or 2 for Lander sync. (default)");
      SerialPrintDebug("delay 3s ----");
      delay(3000);
      SerialPrintDebugln("DONE");
      pushButtonRefresh(PBState);
      SerialPrintDebug("WIFI SETUP\n  Connecting to: ");
      switch (PBState){
        case 1:
          SerialPrintDebugln(WIFI_SSID);
          WiFi.begin(WIFI_SSID, WIFI_PASS);
          break;
        case 2:
          SerialPrintDebugln(WIFI_SSID2);
          WiFi.begin(WIFI_SSID2, WIFI_PASS2);
          break;
        case 3:
          SerialPrintDebugln("ERROR -- button 3 pushed. Changing to DEPLOY");
          changeStateTo(ST_DEPLOY);//change to state 1
          break;
        default:
          SerialPrintDebugln(WIFI_SSID2);
          WiFi.begin(WIFI_SSID2, WIFI_PASS2);
          PBState = 2;
          break;
      }
      SerialPrintDebug("Connecting Wifi...");
      unsigned long startTime = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {
        delay(500);
        SerialPrintDebug(".");
      }
      SerialPrintDebug("    IP address: ");
      SerialPrintDebugln(WiFi.localIP().toString());  // .toString() -> dotted form; without it the IPAddress decays to a raw uint32_t

      DateTime time = rtcExt.now();
      writeLogFile(time.timestamp(DateTime::TIMESTAMP_FULL));
    }


  //------- SD CARD SETUP ----------------------------------------------------------------------------------
    delay(100);
    if (currentState == ST_CONFIG or currentState == ST_DEPLOY or currentState == ST_SEABED or currentState == ST_RELEASE or currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      SerialPrintDebugln("SD INFO");
      //pinMode(19, INPUT_PULLUP); //pullup GPIO2 for SD_MMC mode, you need 1-15kOm resistor connected to GPIO2 and GPIO19
      SD.end();
      delay(500);
      //Check if SD starts correctly, else exit setup
      if (!SD.begin()) {
        SerialPrintDebugln("  Card Mount Failed");
        return;   //ojo amb aquest return --> posar while?
      }
      //Print Card Type
      uint8_t cardType = SD.cardType();
      SerialPrintDebug("  SD Card Type: ");
      switch (cardType) {
        case CARD_NONE:
          SerialPrintDebugln("NO SD CARD ATTACHED");
          return;  // Exit Setup
          break;
        case CARD_MMC:
          SerialPrintDebugln("MMC");
          break;
        case CARD_SD:
          SerialPrintDebugln("SDSC");
          break;
        case CARD_SDHC:
          SerialPrintDebugln("SDHC");
          break;
        default:
          SerialPrintDebugln("UNKNOWN");
          break;
      }
      // Print card info
      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      #ifdef SERIAL_DEBUG
        Serial.printf("  SD Card Size: %lluMB\n", cardSize);
      #endif
    }
  //--------CONF FILE PARAMETERS ---------------------------------------------------------------------------
    if (currentState == ST_CONFIG or currentState == ST_DEPLOY or currentState == ST_SEABED or currentState == ST_RELEASE or currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      getInfoFromConfFile();   // To get all the informations put by the user in this conf file
      char locationSD[64];
      sprintf(locationSD, "/PopUpBuoy_%d", idBuoy);
      char nameFileData[64] = "/dataFile.txt";
      SD_data_filename = (char *)malloc(strlen(locationSD) + strlen(nameFileData) + 1);
      // Comprueba si se asignó memoria correctamente
      if (SD_data_filename != NULL) {
        // Copia locationSD a SD_data_filename
        strcpy(SD_data_filename, locationSD);
        // Concatena nameFileData a SD_data_filename
        strcat(SD_data_filename, nameFileData);
      } else {
        SerialPrintDebug("Error: not possible to allocate the memmory");
      }
    writeLogFile("-----------------//INITIALIZATION\\\\--------------------- ");
    }
   //------- TIME UPDATE FROM UDP SERVER OR LANDER --------------------------------------------------------------------
      char date[10] = "hh:mm:ss";
      rtcExt.now().toString(date);
      writeLogFile("The time on the RTC is " + String(date));

    if (currentState == ST_CONFIG && PBState == 1 && WiFi.status() == WL_CONNECTED) {
      //sincronise time from NTP server
      SerialPrintDebugln("Obtaining time from NTP server");
      timeClient.begin();
      timeClient.update();
      rtcExt.adjust(DateTime(timeClient.getEpochTime()));

      rtcExt.now().toString(date);
      SerialPrintDebug("time changed in RTC DS3231. current time:");
      SerialPrintDebugln(date);

    } else if (currentState == ST_CONFIG && PBState == 2 && WiFi.status() == WL_CONNECTED){
      SerialPrintDebugln("Obtaining time from lander");
      if (!sendHttpGetRequest(idBuoy,GETTIME,releaseFlag,releaseMode,sleeptime_h,sleeptime_m)){
        writeLogFile("Adjustment of RTC time of buoy failed for wrong HTTP request!" );
      }else{
        // Ajustar el RTC con los valores obtenidos
        DateTime newTime(year_lander, month_lander, day_lander, hour_lander, minute_lander, second_lander);
        rtcExt.adjust(newTime);
        writeLogFile("Adjustment of RTC time with the time Lander done." );
      }
      if (!sendHttpGetRequest(idBuoy,GETSYNCTIME,releaseFlag,releaseMode,sleeptime_h,sleeptime_m)){
        writeLogFile("Adjustment of SYNCTIME failed for wrong HTTP request! Setting 9 as default synctime" );
        syncTime = 9;
        eepromSaveSyncTime(syncTime);
      }else{
        // Ajustar el SYNCTIME con los valores obtenidos
        eepromSaveSyncTime(syncTime);
        delay(50);
        writeLogFile("Adjustment of SYNCTIME of buoy " +String(idBuoy)+ " done." );
      }
    }

    // CONFIG is done with the server here, but DEPLOY then waits for PB_1 for as long as the user
    // takes to press it. Release the AP now instead of staying associated all that time.
    wifiShutdown();

  //------- SLEEP MODE SETUP -------------------------------------------------------------------------------
    // Two ways of waking up, by a timmer or by an external iterruption
    //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // pin for the external RTC

  //------- GPS MODULE SETUP -------------------------------------------------------------------------------
    if (currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      SerialPrintDebugln("GPS Module Setup ---->");
      gpsSerialBegin();
      SerialPrintDebug(F("Testing TinyGPSPlus library v. "));
      SerialPrintDebugln(TinyGPSPlus::libraryVersion());
      delay(10);
      SerialPrintDebugln("GPS Module Setup ----> DONE");
    }
  //------- SATELLITE MODULE SETUP (KIM1 or Arribada, whichever is fitted) -----------------------------------
    if (currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      SerialPrintDebugln("Satellite Module Setup ---->");

      // Probes the socket, reads the module ID and cross-checks it against the
      // ID list on the SD card. Everything below goes through sat_module.*, so
      // the rest of the sketch does not care which transmitter is installed.
      while (satModuleDetect() == SAT_UNKNOWN) {
        SerialPrintDebugln("No satellite module answered. Retriying in 1s...");
        delay(1000);
      }

      SerialPrintDebugln(satModuleGetSN());
      delay(delayKIM);
      char logBuf[64];
      snprintf(logBuf, sizeof(logBuf), "%s ID: %s", satModuleName(), satModuleGetID());
      writeLogFile(logBuf);
      SerialPrintDebugln(logBuf);
      delay(delayKIM);
      SerialPrintDebugln("Satellite Module Setup ----> DONE");
    }
 //------- ADC SETUP ---------------------------------------------------------------------------------------
  if (currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
    SerialPrintDebugln("ADC Setup ---->");
    adcSetup();
  }
 //------- FINALISED SETUP --------------------------------------------------------------------------------
  SerialPrintDebugln("\n---------------------------SETUP COMPLETE--------------------------------\n");

  #ifdef SERIAL_DEBUG
    SerialPrintDebug("delay 3s ----");
    delay(3000);
    SerialPrintDebugln("DONE");
    digitalWrite(LED_Y, LOW);
    digitalWrite(LED_G, LOW);
  #endif

}
//------- MAIN LOOP --------------------------------------------------------------------------------------
// Seabed Routine (state ST_SEABED): try a GPS fix (accidental-release check), else connect to
// the pop-up server, request release permission, FTP-download the data and command the release.
void runSeabedRoutine() {
  writeLogFile("Wakeup");
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_Y, LOW);
  digitalWrite(LED_R, LOW);
  bool buoyReleased;
  int sleepTimeState2_h;
  int sleepTimeState2_m;
  // --- TRYING TO FIND GPS  ---
    Counter_FailWIFI = eepromReadCounterWIFIFail();
    if (Counter_FailWIFI==0){  //We just try the gps at the first wifi attempt
      writeLogFile("Trying to find satellites.");
      if (gpsAcquireSatellites()){
        writeLogFile("WARNING! Satellites found. Moving to case 4 - DM.");
        for (int i = 0; i <= 4; i++) {
          digitalWrite(LED_Y, HIGH);
          delay(100);
          digitalWrite(LED_Y, LOW);
          delay(100);
        }
        changeStateTo(ST_DM);
        SleepModeSequence(0, 5, 0, 0);
        return;
      }else{
        writeLogFile("No satellites found. Continue SEABED.");
          for (int i = 0; i <= 4; i++) {
            digitalWrite(LED_R, HIGH);
            delay(100);
            digitalWrite(LED_R, LOW);
            delay(100);
          }
      }
    }
  // --- DOWNLOADING DATA AND SENDING RELEASE COMMAND  ---
    if (!connectToRaspWiFi()) {
      buoyReleased = false;
      writeLogFile("Release of buoy " +String(idBuoy)+ " failed for No-WiFi!");
    }else {
      digitalWrite(LED_R, HIGH);
      if (!sendHttpGetRequest(idBuoy,GETTIME,releaseFlag,releaseMode,sleeptime_h,sleeptime_m)){
        writeLogFile("Adjustment of RTC time of buoy " +String(idBuoy)+ " failed for wrong HTTP request!" );
      }else{
        // Ajustar el RTC con los valores obtenidos
        DateTime newTime(year_lander, month_lander, day_lander, hour_lander, minute_lander, second_lander);
        rtcExt.adjust(newTime);
        writeLogFile("Adjustment of RTC time of buoy " +String(idBuoy)+ " with the time Lander done." );
      }
      if (!sendHttpGetRequest(idBuoy,PERMISSION,releaseFlag,releaseMode,sleeptime_h,sleeptime_m)){
        buoyReleased = false;
        writeLogFile("Release of buoy " +String(idBuoy)+ " failed for wrong HTTP request!");
      }else{
        if (!releaseFlag){
          buoyReleased = false;
          Counter_FailWIFI = 3; //as if it has failed 3 times, sleep till next cycle
          eepromSaveCounterWIFIFail(Counter_FailWIFI);
          writeLogFile("The release request of the buoy " +String(idBuoy)+ " was negative.");
        }else{
          if (connectToFTP() < 0) {
            buoyReleased = false;
            writeLogFile("Connection to FTP failed!");
          }else{
            digitalWrite(LED_Y, HIGH);
            if (downloadAllFilesFTP() < 0) {
              buoyReleased = false;
              writeLogFile("Download of FTP files from buoy " +String(idBuoy)+ " failed.");
            }else{
              digitalWrite(LED_G, HIGH);
              writeLogFile("Files downloaded");
              if (!sendHttpGetRequest(idBuoy,RELEASE,releaseFlag,releaseMode,sleeptime_h,sleeptime_m)){
                buoyReleased = false;
                writeLogFile("Release request of buoy " + String(idBuoy) + " failed.");
              }else{
                unsigned long startTime = millis();
                unsigned long actualTime = 0;
                buoyReleased = true;
                while (WiFi.status() == WL_CONNECTED) { //if we can't connect to the rasp, then all is ok and we can move to phase 4
                  actualTime = millis();
                  if (actualTime - startTime >= maxWIFITimeout) {
                    buoyReleased = false; //if we can still connect to the rasp after timeout that means we are still here --> repeat
                    break;
                  }
                }
              }
            }
          }
        }
      }
    }

  // --- DEFINING SLEEP AND CHANGING STATE  ---
    if (buoyReleased){
      sleepTimeState2_m = 1;
      writeLogFile("Release of buoy " + String(idBuoy)+ " success! Sleeping for " + String (sleepTimeState2_m) + " minutes to reach the surface." );
      switch (releaseMode) {
        case FRM:
          changeStateTo(ST_FRM); //Change state to 6 and save state in eeprom
          writeLogFile("Finished release phase, changing state to FRM as Fast Recovery Mode.");
          break;
        case DM:
          changeStateTo(ST_DM); //Change state to 4 and save state in eeprom
          writeLogFile("Finished release phase, changing state to DM as Drifting Mode");
          break;
        default:
          changeStateTo(ST_DM); //Change state to 4 and save state in eeprom
          writeLogFile("Finished release phase. ERROR reading the release mode. Guessing Drifting Mode (state to DM)");
          break;
      }
      SetCoverageStateTo(0); //first release no transmission of data cause we havent fix the GPS
      SleepModeSequence(0, sleepTimeState2_m, 0, 0); //Enter Sleep Mode
      delay(10);
      return;
    }else {
      Counter_FailWIFI = eepromReadCounterWIFIFail();
      if(Counter_FailWIFI>=2){
        sleepTimeState2_m = 0;
        if (sleeptime_h == 0){
          sleeptime_h =24; //in case no wiffi connect for 3 times (sleeptime_h no initialized)
        }
        sleepTimeState2_h = sleeptime_h;  //here we set the cycle time
        IncrementCounterFailWIFI();
        writeLogFile("Not achieved release phase for 3 WiFi attempts or early request, keeping SEABED. Going to sleep for" + String(sleepTimeState2_h) + " hours and " + String(sleepTimeState2_m) + " minutes to repeat the release.");
        SleepModeSequence(sleepTimeState2_h, sleepTimeState2_m, 0, 1); //Enter Sleep Mode
        return;
      }else{
        sleepTimeState2_m = sleepTimeWifiAttempt;
        sleepTimeState2_h = 0;
        IncrementCounterFailWIFI();
        writeLogFile("Not achieved release phase for " + String(Counter_FailWIFI) + " WiFi attempts, keeping SEABED. Going to sleep for" + String(sleepTimeState2_h) + " hours and " + String(sleepTimeState2_m) + " minutes to repeat the release.");
        SleepModeSequence(sleepTimeState2_h, sleepTimeState2_m, 0, 0); //Enter Sleep Mode
        return;
      }
    }
}

void loop() {

  writeLogFile("-----------------//Rebooting\\\\--------------------- ");

  // read push buttons
  pushButtonRefresh(PBState);

  // State Management
  switch (PBState) {

    case 1:
      SerialPrintDebugln("<---- INITIALISATION PROCEDURE ---->");
      initializationprocedure();
      SerialPrintDebugln("<---- INITIALISATION PROCEDURE ----> DONE");
      SleepModeSequence(0, 0, 5,0);
      break;
    case 2:
      // ATTENTION THIS MUST BE REMOVED, ONLY USED TO GO DIRECTLY TO STATE 4 TO TEST with coverage yes and 2000 seconds
      currentState = ST_DM;
      eepromSaveState(currentState);
      //SetCoverageDurationTo_0();
      eepromSaveTimeCoverage(500);
      SetCoverageStateTo(1);
      SetCounterFailGPSTo_0();
      SetCounterFailWIFITo_0();
      SerialPrintDebugln("You can switch off the board now, buoy ready to start the test from state 4 with no coverage.");
      delay(10000);
      break;
    case 3:
      // ATTENTION THIS MUST BE REMOVED, ONLY USED TO GO DIRECTLY TO STATE 5 TO TEST
      currentState = ST_FRM;
      eepromSaveState(currentState);
      SetCoverageDurationTo_0();
      eepromSaveTimeCoverage(200);
      SetCoverageStateTo(1);
      SetCounterFailGPSTo_0();
      SetCounterFailWIFITo_0();
      SerialPrintDebugln("You can switch off the board now, buoy ready to start the test from state " + String(stateName(currentState)) + " with no coverage.");
      delay(10000);

      break;

    default:
      #ifdef SERIAL_DEBUG   // We notify by led the user, no need if we are not in DEBUG
        SerialPrintDebug(".");
        delay(500);
        digitalWrite(LED_G, LOW);
        delay(500);
      #endif
      break;
  }

  //Production Control
  switch (currentState) {
    case ST_CONFIG:  //CONFIGURATION -- Set up state, in the set up we have set up the RTC time (NTP) and configured the SD, FTP and WiFi
      digitalWrite(LED_R, HIGH);
      writeLogFile("First Boot.");
      SerialPrintDebug("Configuration finished for POP_UP_BUOY at state: ");
      SerialPrintDebugln(stateName(currentState));
      SerialPrintDebugln("Moving to DEPLOY.");
      changeStateTo(ST_DEPLOY);//change to state 1
      break;
    case ST_DEPLOY:  //DEPLOYMENT -- Deployment sleep
      writeLogFile("- sleeping for " +  String(sleeptime_s1_h) + " hours and " + String(sleeptime_s1_m) + " minutes." ); //configure sleep (each pop up buoy will have a different time)
      // wait until button is pressed
      SerialPrintDebugln("Waiting for PB_1 to be pressed to start mission (set to sleep for lander installation)");
      while (digitalRead(PB_1) != false) {
        delay(500);
        digitalWrite(LED_R, !digitalRead(LED_R));
      }
      digitalWrite(LED_R, HIGH);
      changeStateTo(ST_SEABED);//change to state 1
      writeLogFile("PB pressed, changing state to SEABED");
      #ifdef SERIAL_DEBUG
        delay(1000);  //necessary to discharge the intrinsec capacitor of button 1
      #endif
      writeLogFile("Finished deployment phase, changing state to SEABED");
      SleepModeSequence(sleeptime_s1_h, sleeptime_s1_m, 0,1); //Enter Sleep Mode
      delay(10);
      break;

    case ST_SEABED:  //DEEP WATER ROUTINE -- Pre-Launch, ask for permission and download (deep sea routines)
      runSeabedRoutine();
      break;

    // ST_RELEASE (3): unused -- release is handled inside ST_SEABED, so no case here.
    case ST_DM:  //Surface (ocean surface routines)
      writeLogFile("Wakeup");
      // --- INITIALIZING THE STATE 4  ---
      // --- CONFIGURING THE SATELLITE MODULE  ---
        configureKIM();
        writeLogFile(String(satModuleName()) + " power changed to 1000");
      // --- READ EEPROM INFO ABOUT  AND NUMBER OF FILES IN DATAFILE AND THE ACTUAL RowProgress  ---
        Decimal_CoverageDuration = eepromReadCoverageDuration();  // Duration on 2 bytes ; 1 byte would be too short for a number of seconds
        SerialPrintDebugln(" Time of coverage from the comming satellite : " + String(Decimal_CoverageDuration) + String(" sec"));
        countLinesInDataFile(); // To get how many lines there are in this file
        readSuccessFile(); // To get the actual RowProgress

      // --- DEFINING IF THERE IS ARGOS COVERAGE AND HOW THE CODE MUST ANSWER ---
        CoverageState = eepromReadCoverageState();  // Read if the buoy is in a time where satellites are passing

        if (CoverageState == 0) {        // no coverage so only sending the GPS data and going back to sleep
          timeSending = timetransm_GPS_noArg_s;              // sec of sending --> if 90 => 3 kineis MSG
          fileSendingTime = 0;
          writeLogFile("No ARGOS coverage, sending 3 GPS messages and going back to sleep");

        } else if (CoverageState == 1) {                                   // There is coverage
          if (RowProgress>MaxRowDataFile){                       // End of datafile or not Found (MaxRowDataFile is : 0 and Rowfile is 1 or more) (if we can't open both RowProgress is 1 and MaxRowDataFile is 0)
            timeSending = timetransm_GPS_s;                        // Just transmitting position all time
            fileSendingTime = 0;
            waitSendingTime = (Decimal_CoverageDuration - timeSending) / 2; // we will wait to ensure the GPS is sent in the midle of Argos
            writeLogFile("Argos coverage OK. DataFile completely sent or not found. Sending GPS for " + String(timeSending) + " seconds. Then go back to sleep.");
          }else{
            timeSending = timetransm_GPS_s;                                // XX sec of sending GPS
            fileSendingTime = (Decimal_CoverageDuration - timeSending) / 2;  // In this case we must send the file but GPS is sent at the middle of the coverage so we sent the file before and after
            writeLogFile("Argos coverage OK, sending data for " + String(fileSendingTime) + " seconds twice and GPS for " + String(timeSending) + " seconds. Then go back to sleep.");
          }

        }

      // --- OBTAINING THE GPS AND ADC DATA ---
        adcAcquireData(ADCreadHex);
        gpsAcquireData(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);
        gpsSave(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);

      // --- SENDING MESSAGES PART ---
        if (CoverageState == 0 ) {
          SendGPSMessage(timeSending);   // We don't care when the message is sent because there's no ARGOS coverage
        } else {
          if (fileSendingTime>0 && RowProgress<MaxRowDataFile) {
            readSuccessFile();
            SendFileKim(fileSendingTime);  // file sent before the GPS data
          } else { // End of datafile or not Found
            writeLogFile("DataFile completely sent or not found. Light sleep for " + String(waitSendingTime) + " to ensure the GPS message is sent at the midle of the coverage.");
            goToSleep(waitSendingTime);
          }
          SendGPSMessage(timeSending);   // The GPS is sent at the middle of the coverage --> better chance to be received by satellites
          if (fileSendingTime>0 && RowProgress<MaxRowDataFile) {
            SendFileKim(fileSendingTime);  // file sent after the GPS data
          } else {
            // No need to sleep again! Directly to sleep to avid innecessary consumption
          }
        }


        writeLogFile("End of " + String(satModuleName()) + " transmissions.");
        delay(10);

      // --- SATELLITE PASS PREDICTION --- pass prediction only if GPS fix
        runSatellitePassPrediction(false);

      // --- DEFINING THE MAXIUM TIME BETWEEN TWO GPS SENDING ---
        if (secondsBeforeNextStatellite > max_sleep_time_s) {  // in seconds
          secondsBeforeNextStatellite = max_sleep_time_s;      // to fix the time to sleep to YY min so that even when there is no ARGOS coverage you transmitt for boat recovery
          writeLogFile("Time to sleep too long. Recovery messages needed. Changing coverage_state to 0 and sleeping for "+ String(max_sleep_time_s) +" sec.");
          SetCoverageStateTo(0);
        }

      // --- CHANGING THE BUOY STATE AND SLEEP ---
        if ( Vin_ADC>Bat_critlevel){  //Battery still ok
          writeLogFile("Going to sleep for " + String(secondsBeforeNextStatellite) + " sec.");
          sleepSecondsAndGoTo(secondsBeforeNextStatellite, ST_DM);
        }else{  //
          writeLogFile("BATTERY ALERT! Changing to LOWPWR and going to sleep for " + String(secondsBeforeNextStatellite) + " sec.");
          sleepSecondsAndGoTo(secondsBeforeNextStatellite, ST_LOWPWR);
        }
      break;
    case ST_LOWPWR:  //LOW-power

      writeLogFile("Wakeup");
      // --- INITIALIZING THE STATE 5  ---
      // --- CONFIGURING THE SATELLITE MODULE  ---
        configureKIM();
        writeLogFile(String(satModuleName()) + " power changed to 1000");
      // --- READ EEPROM INFO ABOUT DURATION ---
        Decimal_CoverageDuration = eepromReadCoverageDuration();  // Duration on 2 bytes ; 1 byte would be too short for a number of seconds
        SerialPrintDebugln(" Time of coverage from the comming satellite : " + String(Decimal_CoverageDuration) + String(" sec"));

      // --- DEFINING IF THERE IS ARGOS COVERAGE AND HOW THE CODE MUST ANSWER ---
        CoverageState = eepromReadCoverageState();  // Read if the buoy is in a time where satellites are passing -- IN LOWBAT_MODE always 1 - no recovery messages

        if (CoverageState == 0) {        // no ARGOS coverage  -- if no GPS fix in 5 this can happen
          timeSending = timetransm_GPS_noArg_s;              // All time dedicated to transmitt GPS
          writeLogFile("No ARGOS coverage, sending 3 GPS messages and going back to sleep");

        } else if (CoverageState == 1) {                                   // There is coverage
          timeSending = Decimal_CoverageDuration;                          // All time dedicated to transmitt GPS
          writeLogFile("Argos coverage OK, sending GPS data for " + String(timeSending) + " seconds. Then go back to sleep.");
        }

      // --- OBTAINING THE GPS AND ADC DATA AND TRANSMITTING ---
        gpsAcquireData(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);
        adcAcquireData(ADCreadHex);
        gpsSave(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);

      // --- SENDING MESSAGES PART --- this can be moved down

        SendGPSMessage(timeSending);   // No need for delay. If no Argoscoverage, don't care when its sent. If argos coverage, all time sending GPS data so also in the midle.
        writeLogFile("End of " + String(satModuleName()) + " transmissions.");
        delay(10);

      // --- SATELLITE PASS PREDICTION --- pass prediction only if GPS fix
        runSatellitePassPrediction(true);

      // --- DEFINING THE MAXIUM TIME BETWEEN TWO GPS SENDING ---  NOT USED IN LOWBAT_MODE
      // --- CHANGING THE BUOY STATE AND SLEEP ---
        if ( Vin_ADC>Bat_critlevel){  //Battery still ok
          writeLogFile("Battery OK again. Changing to DM and going to sleep for " + String(secondsBeforeNextStatellite) + " sec.");
          sleepSecondsAndGoTo(secondsBeforeNextStatellite, ST_DM);
        }else{  //
          writeLogFile("BATTERY ALERT! Going to sleep for " + String(secondsBeforeNextStatellite) + " sec.");
          sleepSecondsAndGoTo(secondsBeforeNextStatellite, ST_LOWPWR);
        }
      break;
    case ST_FRM:  //Fast Recovery Mode
      writeLogFile("Wakeup");
      // --- INITIALIZING THE STATE 6  ---
      // --- CONFIGURING THE SATELLITE MODULE  ---
        configureKIM();
      // --- SENDING REGULAR UPDATED  MESSAGES ---
        adcAcquireData(ADCreadHex);  //acquiring bat for the 1st time
        unsigned long initTime;
        initTime = millis();
        timeSending = 30; //Sending just one repetition
        maxGPSTimeout = 60000; //here better 60
        // >= (not >) so the loop only leaves FRM when the battery is STRICTLY below critical,
        // matching the post-loop check (else if Vin_ADC < Bat_critlevel -> LOWPWR). With > and
        // Bat_critlevel=0, a single Vin_ADC==0 reading exited the loop, ran SPP and fell through to
        // the "ERROR -> DM" branch (equality landed in the gap between > here and < there).
        while((millis() - initTime < maxFRM*3600*1000) && (Vin_ADC >= Bat_critlevel)){  //maxFRM en horas
          adcAcquireData(ADCreadHex);
          gpsAcquireData(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);
          gpsSave(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);
          writeLogFile("Sending updated GPS position");
          SendGPSMessage(timeSending);

      // --- USV DATA UPLOAD --- offer SD data to a BlueBoat if one is in range
        #ifdef FRM_USV_UPLOAD
          int ret = tryUploadDataToUSV();
          if (ret == 0) {
            // Upload confirmed. Stay in FRM (do NOT drop to DM) and sleep 4 min,
            // then run the full upload cycle again — keeps offering the server
            // anything new that accumulates while the buoy stays with the USV.
            // sleepSecondsAndGoTo deep-sleeps (reboots), so it doesn't return.
            writeLogFile("USV upload complete. Staying in FRM, sleeping 4 min before next cycle.");
            sleepSecondsAndGoTo(4 * 60, ST_FRM);
          }
        #endif
        }
      // --- SATELLITE PASS PREDICTION --- pass prediction only if GPS fix
        runSatellitePassPrediction(false);
      // --- DEFINING THE MAXIUM TIME BETWEEN TWO GPS SENDING ---
        if (secondsBeforeNextStatellite > max_sleep_time_s) {  // in seconds
          secondsBeforeNextStatellite = max_sleep_time_s;      // to fix the time to sleep to YY min so that even when there is no ARGOS coverage you transmitt for boat recovery
          writeLogFile("Time to sleep too long. Recovery messages needed. Changing coverage_state to 0 and sleeping for "+ String(max_sleep_time_s) +" sec.");
          SetCoverageStateTo(0);
        }

      // --- MOVING TO NEXT STATUS ---
        if((millis() - initTime > maxFRM*3600*1000)) {
          writeLogFile("Fast Recovery Mode timeout! Not recovered for " + String(maxFRM) + " hours, so sleeping for " + String(secondsBeforeNextStatellite) + "seconds and moving to Drifting Mode at DM.");
          sleepSecondsAndGoTo(secondsBeforeNextStatellite, ST_DM);
        }else if(Vin_ADC < Bat_critlevel){
          writeLogFile("BATTERY ALERT! Sleeping for " + String(secondsBeforeNextStatellite) + " seconds and changing to LOWPWR.");
          sleepSecondsAndGoTo(secondsBeforeNextStatellite, ST_LOWPWR);
        }else{
          writeLogFile("ERROR! Sleeping for 5 minutes and moving to DM.");
          sleepSecondsAndGoTo(300, ST_DM);
        }
      break;
    default:
      SerialPrintDebugln("State ERROR - No state detected.");
      delay(1000);
      break;
  }
}

//------- FUNCTIONS FOR BASIC FUNCTIONALITY ---------------------------------------------------------------
void initializationprocedure(){
  eepromInitState();  // initialise epprom state to 0
  delay(10);  //Modify this two deletes to one
  deleteFile(Log_filename); // Deletes the log file with events. File will be created again during the program progress
  deleteFile(GPSfilename); //Deletes track file with GPS coordinates. File will be created again during the program progres
  deleteFile(SD_progress_filename); //Deletes progress file with file send steps. File will be created again during the program progres -> Should start with 1:0
  createProgressFile();  //create the file starting from 1:0 progress
  ConnectPeripherals(true, GPS_KIM);
  configureKIM();      //Configure KIM1
  SetCounterFailGPSTo_0();
  SetCounterFailWIFITo_0();
}
void pushButtonRefresh(int &pushedButton) {

  if (digitalRead(PB_1) == false) {

    #ifdef SERIAL_DEBUG
      SerialPrintDebugln("PB_1  pressed.");
      digitalWrite(LED_G, HIGH);  // to be sure visually that the button has been pushed when there is no serial monitor
      delay(2000);
      digitalWrite(LED_G, LOW);
    #endif
    pushedButton = 1;
  }
  else if (digitalRead(PB_2) == false) {
    #ifdef SERIAL_DEBUG
      SerialPrintDebugln("PB_2  pressed.");
      digitalWrite(LED_Y, HIGH); // to be sure visually that the button has been pushed when there is no serial monitor
      delay(2000);
      digitalWrite(LED_Y, LOW);
    #endif
    pushedButton = 2;
  }

  else if (digitalRead(PB_3) == false) {
    #ifdef SERIAL_DEBUG
      SerialPrintDebugln("PB_3  pressed.");
      digitalWrite(LED_R, HIGH); // to be sure visually that the button has been pushed when there is no serial monitor
      delay(2000);
      digitalWrite(LED_R, LOW);
    #endif
    pushedButton = 3;
  }
  // No case
  else {
    #ifdef SERIAL_DEBUG
    SerialPrintDebugln("No PB Pressed");
    #endif
    pushedButton = 0;
  }

  // wait until button has been released
  while ((digitalRead(PB_1) != true) or (digitalRead(PB_2) != true) or (digitalRead(PB_3) != true)) {

    delay(50);
  }
}
// Serial debug + SD logging helpers now live in logging.h / logging.cpp
bool deleteFile(const char *filename) {

  if (SD.exists(filename)) {
    SD.remove(filename);
    SerialPrintDebugln(String(filename) + " removed.");
    return true;
  } else {
    SerialPrintDebugln(String(filename) + " doesn't exist.");
    return false;
  }
}
void createProgressFile() {
  progressDataFileSD = SD.open(SD_progress_filename, FILE_WRITE);

  if (progressDataFileSD) {
    // Escribe "1:0" seguido de un salto de línea
    progressDataFileSD.println("1:0");

    // Cierra el archivo
    progressDataFileSD.close();

    SerialPrintDebugln("Progressfile initiated properly.");
  } else {
    SerialPrintDebugln("Error creating progressfile.");
  }
}
// EEPROM state-store helpers now live in eeprom_store.h / eeprom_store.cpp
// KIM-TX config/message part (configureKIM, sendGPSviaKIM, maskGPS) now in satellite_tx.h + satellite_tx.cpp
// SPP module (runSatellitePassPrediction, NextSatellite) now in satellite_spp.h + satellite_spp.cpp
// EEPROM state-store helpers now live in eeprom_store.h / eeprom_store.cpp
// ChangeSecondsInHoursAndMinutes + sleepSecondsAndGoTo now in power_sleep.h / power_sleep.cpp
// KIM-TX send part (SendGPSMessage, SendDataMessage, readSuccessFile, splitLineProgressFile, splitLineDataFile) now in satellite_tx.h + satellite_tx.cpp
// Config module (getInfoFromConfFile + its SD /conf.txt parsing) now in config.h + config.cpp
//------- FUNCTIONS FOR AOP TABLE GENERATION-----------------------------------------------------------------------
// SPP AOP-table helpers (parseLine, readSatelliteData, printAopTable) now in satellite_spp.h + satellite_spp.cpp
