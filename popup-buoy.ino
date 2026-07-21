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
#include "satellite_spp.h"
#include "satellite_tx.h"
#include "Arduino.h"
#include "SD.h"
#include <RTClib.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "ESP32_FTPClient.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "KIM.h"
#include <EEPROM.h>
#include <Wire.h>


//------ Configuration FTP server -------------------------------------------------------------------------------------
  ESP32_FTPClient ftp(SECRET_FTP_SERVER_IP, SECRET_FTP_SERVER_USER, SECRET_FTP_SERVER_PASS);

//------ Configuration for GPS module ---------------------------------------------------------------------------------
  TinyGPSPlus gps;                                 // The TinyGPSPlus object
  SoftwareSerial gpsSerial(RXPin_GPS, TXPin_GPS);  // The serial connection to the GPS device

//------ Configuration for KINEIS module ------------------------------------------------------------------------------
  HardwareSerial kimSerial(2);  // hard coded no library
  KIM KIM(&kimSerial);          //with library

//------ Define Kineis Transmission Parameters ------------------------------------------------------------------------
  char PWR2[10] = "1000";  // Rise of power from 500 to 1000 -- These parameters are saved in RAM, not defined anymore
  char PWR3[10] = "100";
  char AFMT[] ="1";     // Enable standard kim messages -- These parameters are saved in RAM, not defined anymore
  extern const int delayKIM = 10;  //between parmeters set (extern: shared with satellite_tx.cpp)
  char kineisMessage[27];  // declared globally to avoid errors
  char kineisdataMessage[47];

//------ Define Kineis SPP Parameters ---------------------------------------------------------------------------------
  char *new_line;  //Variable used to send the data from the SD file
  int secondsBeforeNextStatellite;
  int hoursBeforeNextStatellite;
  int minutesBeforeNextStatellite;
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

//------ Define GPS Acquiring Parameters -------------------------------------------------------------------------------
  double gpsLat, gpsLong;
  uint8_t gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond;
  uint16_t gpsYear;
  bool gpsFix;
  uint32_t epochTime;
  int maxGPSTimeout;

//------ Define Variables for the Kineis communication -----------------------------------------------------------------
  int NbrMsgToSend;
  int RowProgress;
  int nbrSendingProgress;
  int MaxRowDataFile;
  int MaxNbrMsgSendingDataFile;

//------ Basic definitions ---------------------------------------------------------------------------------------------
  int currentState = INITIAL_STATE;  // current state of pop-up-buoy (0 submerged, 1 surfacing,...)
  int PBState = 0;                   // state of push buttons
  int idBuoy;
  // ActionType and ReleaseMode enums now defined in wifi_http.h
  int releaseFlag;
  ReleaseMode releaseMode;
  int sleeptime_h;
  int sleeptime_m;
//------ Timming definitions ---------------------------------------------------------------------------------------------
  int sleeptime_s1_h; //Time to Sleep state 1 (from config to deep sea) in hours
  int sleeptime_s1_m; //Time to Sleep state 1 (from config to deep sea) in min
  int sleeptime_errorGPS_s; //Time to sleep when the GPS can't fix- 1st time (s)
  int sleeptime_errorGPS_recurrent_s; //Time to sleep when the GPS can't fix for multiple times (s)
  int max_sleep_time_s; //Maximum surface sleep time in s at any condition (to ensure the recovery)
  int timetransm_GPS_s; //Time for normal GPS transmission, minimum --> Minimum duration --> 300 s =10 messages .. now is 2 messages
  int timetransm_GPS_noArg_s; //Time for GPS transmission, no ARGOS coverage (default 90 -> 3 messages)
  unsigned long maxFRM; //Max time in stage 6
  int maxWIFITimeout;
  int sleepTimeWifiAttempt;
  int year_lander, month_lander, day_lander, hour_lander, minute_lander, second_lander;
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

//------ Definitions for naming of the SD files ------------------------------------------------------------------------
  const char *GPSfilename = "/GPS_track.csv";  // file with all the GPS data
  File GpsTrackFile;
  const char *Log_filename = "/LogFile.txt";  // Log file
  File LogFile;
  char *SD_data_filename; // File containing all the data which are going to be sent
  File datamsgSD;
  const char *SD_progress_filename = "/progressFile.txt";  // File containing the progress of sending the file, to be used to get where we are in the file with data
  File progressDataFileSD;
  const char *SD_config_filename = "/conf.txt";  // File with all the config, only thing that will be given to the SD card, so that we won't have to change the code
  File ConfigFileSD;
  const char *AOPfilename = "/AOP.txt";  // file with all the GPS data
  File AOPFile;


//------ Definitions ADC read  -----------------------------------------------------------------------------------------
  // ADC resolution now lives in adc.cpp (ADC_RESOLUTION)
  char ADCreadHex[3]; // Buffer para almacenar el valor hexadecimal
  float Vin_ADC;  //Battery voltage

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

      if (rtcExt.lostPower()) {  // If RTC loses batery power update time with last compilation date time
        // this will adjust to the date and time at compilation
        rtcExt.adjust(DateTime(F(__DATE__), F(__TIME__)));
        SerialPrintDebugln("time adjust");   //comment rtcExt.edjust and put here a print that should be adjust
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
      SerialPrintDebugln(WiFi.localIP());

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

  //------- SLEEP MODE SETUP -------------------------------------------------------------------------------
    // Two ways of waking up, by a timmer or by an external iterruption
    //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // pin for the external RTC

  //------- GPS MODULE SETUP -------------------------------------------------------------------------------
    if (currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      SerialPrintDebugln("GPS Module Setup ---->");
      gpsSerial.begin(GPSBaud);
      SerialPrintDebug(F("Testing TinyGPSPlus library v. "));
      SerialPrintDebugln(TinyGPSPlus::libraryVersion());
      delay(10);
      SerialPrintDebugln("GPS Module Setup ----> DONE");
    }
  //------- KIM MODULE SETUP -------------------------------------------------------------------------------
    if (currentState == ST_DM or currentState == ST_LOWPWR or currentState == ST_FRM) {
      SerialPrintDebugln("KIM Module Setup ---->");
      while (!KIM.check()) {
        SerialPrintDebugln("Failed connexion to KIM module. Retriying in 3s...");
        delay(1000);
      }
      SerialPrintDebugln(KIM.get_SN());
      delay(delayKIM);
      SerialPrintDebugln(KIM.get_ID());
      char logBuf[64];
      snprintf(logBuf, sizeof(logBuf), "KIM ID: %s", KIM.get_ID());
      writeLogFile(logBuf);
      delay(delayKIM);
      SerialPrintDebugln(KIM.get_PWR());
      delay(delayKIM);
      SerialPrintDebugln(KIM.get_AFMT());
      delay(delayKIM);
      SerialPrintDebugln("KIM Module Setup ----> DONE");
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
      eepromSaveTimeCoverage(50);
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
            break;
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
          break;
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
            break;
          }else{
            sleepTimeState2_m = sleepTimeWifiAttempt;
            sleepTimeState2_h = 0;
            IncrementCounterFailWIFI();
            writeLogFile("Not achieved release phase for " + String(Counter_FailWIFI) + " WiFi attempts, keeping SEABED. Going to sleep for" + String(sleepTimeState2_h) + " hours and " + String(sleepTimeState2_m) + " minutes to repeat the release.");
            SleepModeSequence(sleepTimeState2_h, sleepTimeState2_m, 0, 0); //Enter Sleep Mode
            break;
          }
        }
      break;

    // ST_RELEASE (3): unused -- release is handled inside ST_SEABED, so no case here.
    case ST_DM:  //Surface (ocean surface routines)
      writeLogFile("Wakeup");
      // --- INITIALIZING THE STATE 4  ---
      // --- CONFIGURING KIM  ---
        configureKIM();
        writeLogFile("KIM power changed to 1000");
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


        writeLogFile("End of KIM transmissions.");
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
          ChangeSecondsInHoursAndMinutes(&secondsBeforeNextStatellite, &minutesBeforeNextStatellite, &hoursBeforeNextStatellite); // Conversion of the time needed for the sleeping time
          changeStateTo(ST_DM);
          writeLogFile("Entering Sleep mode");
          SleepModeSequence(hoursBeforeNextStatellite, minutesBeforeNextStatellite, secondsBeforeNextStatellite, 0);
          delay(10);
        }else{  //
          writeLogFile("BATTERY ALERT! Changing to LOWPWR and going to sleep for " + String(secondsBeforeNextStatellite) + " sec.");
          ChangeSecondsInHoursAndMinutes(&secondsBeforeNextStatellite, &minutesBeforeNextStatellite, &hoursBeforeNextStatellite); // Conversion of the time needed for the sleeping time
          changeStateTo(ST_LOWPWR);
          writeLogFile("Entering Sleep mode");
          SleepModeSequence(hoursBeforeNextStatellite, minutesBeforeNextStatellite, secondsBeforeNextStatellite, 0);
          delay(10);
        }
      break;
    case ST_LOWPWR:  //LOW-power

      writeLogFile("Wakeup");
      // --- INITIALIZING THE STATE 5  ---
      // --- CONFIGURING KIM  ---
        configureKIM();
        writeLogFile("KIM power changed to 1000");
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
        writeLogFile("End of KIM transmissions.");
        delay(10);

      // --- SATELLITE PASS PREDICTION --- pass prediction only if GPS fix
        runSatellitePassPrediction(true);

      // --- DEFINING THE MAXIUM TIME BETWEEN TWO GPS SENDING ---  NOT USED IN LOWBAT_MODE
      // --- CHANGING THE BUOY STATE AND SLEEP ---
        if ( Vin_ADC>Bat_critlevel){  //Battery still ok
          writeLogFile("Battery OK again. Changing to DM and going to sleep for " + String(secondsBeforeNextStatellite) + " sec.");
          ChangeSecondsInHoursAndMinutes(&secondsBeforeNextStatellite, &minutesBeforeNextStatellite, &hoursBeforeNextStatellite); // Conversion of the time needed for the sleeping time
          changeStateTo(ST_DM);
          writeLogFile("Entering Sleep mode");
          SleepModeSequence(hoursBeforeNextStatellite, minutesBeforeNextStatellite, secondsBeforeNextStatellite, 0);
          delay(10);
        }else{  //
          writeLogFile("BATTERY ALERT! Going to sleep for " + String(secondsBeforeNextStatellite) + " sec.");
          ChangeSecondsInHoursAndMinutes(&secondsBeforeNextStatellite, &minutesBeforeNextStatellite, &hoursBeforeNextStatellite); // Conversion of the time needed for the sleeping time
          changeStateTo(ST_LOWPWR);
          writeLogFile("Entering Sleep mode");
          SleepModeSequence(hoursBeforeNextStatellite, minutesBeforeNextStatellite, secondsBeforeNextStatellite, 0);
          delay(10);
        }
      break;
    case ST_FRM:  //Fast Recovery Mode
      writeLogFile("Wakeup");
      // --- INITIALIZING THE STATE 6  ---
      // --- CONFIGURING KIM  ---
        configureKIM();
      // --- SENDING REGULAR UPDATED  MESSAGES ---
        adcAcquireData(ADCreadHex);  //acquiring bat for the 1st time
        unsigned long initTime;
        initTime = millis();
        timeSending = 30; //Sending just one repetition
        maxGPSTimeout = 60000; //here better 60
        while((millis() - initTime < maxFRM*3600*1000) && (Vin_ADC > Bat_critlevel)){  //maxFRM en horas
          adcAcquireData(ADCreadHex);
          gpsAcquireData(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);
          gpsSave(gpsLat, gpsLong, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, epochTime, gpsFix);
          writeLogFile("Sending updated GPS position");
          SendGPSMessage(timeSending);
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
          changeStateTo(ST_DM);
          ChangeSecondsInHoursAndMinutes(&secondsBeforeNextStatellite, &minutesBeforeNextStatellite, &hoursBeforeNextStatellite); // Conversion of the time needed for the sleeping time
          writeLogFile("Entering Sleep mode");
          SleepModeSequence(hoursBeforeNextStatellite, minutesBeforeNextStatellite, secondsBeforeNextStatellite, 0);
          delay(10);
        }else if(Vin_ADC < Bat_critlevel){
          writeLogFile("BATTERY ALERT! Sleeping for " + String(secondsBeforeNextStatellite) + " seconds and changing to LOWPWR.");
          changeStateTo(ST_LOWPWR);
          ChangeSecondsInHoursAndMinutes(&secondsBeforeNextStatellite, &minutesBeforeNextStatellite, &hoursBeforeNextStatellite); // Conversion of the time needed for the sleeping time
          writeLogFile("Entering Sleep mode");
          SleepModeSequence(hoursBeforeNextStatellite, minutesBeforeNextStatellite, secondsBeforeNextStatellite, 0);
          delay(10);
        }else{
          writeLogFile("ERROR! Sleeping for 5 minutes and moving to DM.");
          changeStateTo(ST_DM);
          writeLogFile("Entering Sleep mode");
          SleepModeSequence(0, 5, 0, 0);
          delay(10);
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
void ChangeSecondsInHoursAndMinutes(int *seconds, int *minutes, int *hours) {
  *hours = *seconds / 3600;           // Conversion en heures
  *minutes = (*seconds % 3600) / 60;  // Conversion en minutes
  *seconds = (*seconds % 3600) % 60;  // Conversion en secondes sans les heures et les minutes
}
// KIM-TX send part (SendGPSMessage, SendDataMessage, readSuccessFile, splitLineProgressFile, splitLineDataFile) now in satellite_tx.h + satellite_tx.cpp
void splitLineSuccessFile(const String &line, char *variableName, int &data) {
  int separatorIndex = line.indexOf('=');  // The line will be cut by the ":" caracter. To change the file , only change here the caracter.

  if (separatorIndex != -1) {
    String VariableNameStr = line.substring(0, separatorIndex);  // This part is to get the name of the variable
    strcpy(variableName, VariableNameStr.c_str());

    String dataStr = line.substring(separatorIndex + 1);  // This part is to get the data
    data = dataStr.toInt();
  }
}
// KIM-TX data-file part (GetLineDataFile, SendFileKim, SaveInProgressFile, countLinesInDataFile) now in satellite_tx.h + satellite_tx.cpp
void getInfoFromConfFile() {

  // This function is used to get every variable we need so that we won't have to change the code

  ConfigFileSD = SD.open(SD_config_filename, FILE_READ);  // Opening the file ConfFile
  if (!ConfigFileSD) {                                    // Cheking if the file is open
    SerialPrintDebugln(String(SD_config_filename) + " couldn't be opened");
  } else {
    SerialPrintDebugln(String(SD_config_filename) + " has been opened");

    while (ConfigFileSD.available()) {  // Looping in the file as long as there are some data in it

      String line = ConfigFileSD.readStringUntil('\n');
      char NameOfVariable[256];
      int DataFromVariable;

      splitLineSuccessFile(line, NameOfVariable, DataFromVariable);

      String VariableNameStr = String(NameOfVariable);
      //SerialPrintDebugln(VariableNameStr);

      if (VariableNameStr == "NumberOfSendingEachLineFromData") {
        MaxNbrMsgSendingDataFile = DataFromVariable;
        SerialPrintDebugln("Number of times transmitting each data line: " + String(DataFromVariable));
      }

      if (VariableNameStr == "idBuoy") {
        idBuoy = DataFromVariable;
        writeLogFile("Id Buoy: " + String(DataFromVariable));
      }

      if (VariableNameStr == "MAX_GPS_TIMEOUT") {
        maxGPSTimeout = DataFromVariable;
        SerialPrintDebugln("Maximum Timeout of GPS: " + String(DataFromVariable) + " miliseconds");
      }

      if (VariableNameStr == "MAX_WIFI_TIMEOUT") {
        maxWIFITimeout = DataFromVariable;
        SerialPrintDebugln("Maximum Timeout of WIFI: " + String(DataFromVariable) + " miliseconds");
      }

      if (VariableNameStr == "TIME_TO_SLEEP_STATE1_h") {
        sleeptime_s1_h = DataFromVariable;
        SerialPrintDebugln("Time to Sleep state 1 (from config to deep sea): " + String(DataFromVariable) + " hours");
      }

      if (VariableNameStr == "TIME_TO_SLEEP_STATE1_m") {
        sleeptime_s1_m = DataFromVariable;
        SerialPrintDebugln("Time to Sleep state 1 (from config to deep sea): " + String(DataFromVariable) + " minutes");
      }

      if (VariableNameStr == "TIME_TO_SLEEP_ERROR_WIFI_m") {
        sleepTimeWifiAttempt = DataFromVariable;
        SerialPrintDebugln("Time to Sleep after WiFi attempt: " + String(DataFromVariable) + " minutes");
      }

      if (VariableNameStr == "TIME_TO_SLEEP_ERROR_GPS_s") {
        sleeptime_errorGPS_s = DataFromVariable;
        SerialPrintDebugln("Time to sleep when the GPS can't fix- 1st time: " + String(DataFromVariable) + " seconds");
      }

      if (VariableNameStr == "TIME_TO_SLEEP_ERROR_GPS_RECURRENT_s") {
        sleeptime_errorGPS_recurrent_s = DataFromVariable;
        SerialPrintDebugln("Time to sleep when the GPS can't fix for multiple times: " + String(DataFromVariable) + " seconds");
      }

      if (VariableNameStr == "MAX_SLEEP_TIME_s") {
        max_sleep_time_s = DataFromVariable;
        SerialPrintDebugln("Maximum surface sleep time in s at any condition (to ensure the recovery): " + String(DataFromVariable) + " seconds");
      }

      if (VariableNameStr == "TRANSMISSION_GPS_s") {
        timetransm_GPS_s = DataFromVariable;
        SerialPrintDebugln("Time for normal GPS transmission: " + String(DataFromVariable) + " seconds");
      }

      if (VariableNameStr == "TRANSMISSION_GPS_NOARG_s") {
        timetransm_GPS_noArg_s = DataFromVariable;
        SerialPrintDebugln("Time for GPS transmission, no ARGOS coverage: " + String(DataFromVariable) + " seconds");
      }

      if (VariableNameStr == "MAX_FRM_TIME_h") {
        maxFRM = DataFromVariable;
        SerialPrintDebugln("Max time at stage 6: " + String(DataFromVariable) + " hours");
      }

      if (VariableNameStr == "MinElev") {  // NUEVA VARIABLE FLOAT
        MinElev = static_cast<float>(DataFromVariable);  // Conversión explícita
        writeLogFile("Minimum Elevation: " + String(MinElev));
      }

      if (VariableNameStr == "PWR2") {  // LEEMOS COMO ENTERO Y LO CONVERTIMOS A CHAR[]
        sprintf(PWR2, "%d", DataFromVariable);
        writeLogFile("PWR2: " + String(PWR2));
      }

      if (VariableNameStr == "PWR3") {  // LEEMOS COMO ENTERO Y LO CONVERTIMOS A CHAR[]
        sprintf(PWR3, "%d", DataFromVariable);
        writeLogFile("PWR3: " + String(PWR3));
      }


      if (VariableNameStr == "FILE_BLINK_LED") {
        fileBlinkLed = DataFromVariable;
        SerialPrintDebugln("Debug FTP file download with LEDs: " + String(DataFromVariable));
      }

      if (VariableNameStr == "BAT_CRIT_LEVEL") {  // NUEVA VARIABLE FLOAT
        Bat_critlevel = static_cast<float>(DataFromVariable)/1000;  // Conversión explícita
        writeLogFile("Battery critical lebel: " + String(Bat_critlevel));
      }

      // To add other lines in the file, just follow the same architecture with the "=" in the middle and add here an else if with the right condition
    }
  }
  ConfigFileSD.close();
}
//------- FUNCTIONS FOR AOP TABLE GENERATION-----------------------------------------------------------------------
// SPP AOP-table helpers (parseLine, readSatelliteData, printAopTable) now in satellite_spp.h + satellite_spp.cpp
