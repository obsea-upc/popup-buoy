/*******************************************************************************
*   00_MasterPoopUpBuoy.ino
*
*   This file contains the master-code for the PopUpBuoys
*
*   HARDWARE:
*   - ESP32-Wroom-32 (EZSBC)
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
*    SCreations.bcn & Matias Carandell (UPC)


WORKING PROGRESS
1. parametres de KIM transmission - power ¿? necessary?
2. Calibrate battey read

FUTURE IMPROVEMENTS
1. Change to internal RTC? calibrate the timmings -- Utilitzar calibració DAN. Utilutzar gps.time per SPP i al log fer algo 
2. Implement a board without intermediate boards. No Evaluation boards, all solded.


******************************************************************************/

#include "conf.h"
#include "secrets.h"
#include "Arduino.h"
#include "SD.h"
#include <RTClib.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>  // included for the realese command to the rasp
#include "ESP32_FTPClient.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "KIM.h"
#include <EEPROM.h>
#include <Wire.h>
#include <FastCRC.h>
#include "previpass.h"

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
  const int delayKIM = 10;  //between parmeters set
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
  enum ActionType {
    RELEASE = 1,
    PERMISSION = 2,
    GETTIME = 3,
    GETSYNCTIME = 4
  };
  enum ReleaseMode {
    FRM,  // FAST RECOVERY MODE
    DM    // DRIFTING MODE
  };
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
  int FRMsleepTime_s; //Time to sleep at stage 6 between transmissions
  int FRMsleepTime_fail_s; //Time to sleep at stage 6 when GPS fail
  unsigned long maxFRM; //Max time in stage 6
  int maxWIFITimeout;
  int sleepTimeWifiAttempt;
  int year_lander, month_lander, day_lander, hour_lander, minute_lander, second_lander;
  int syncTime;
  int fileBlinkLed = false;

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
  const int ADC_resolution = 8; // Resolución del ADC en bits (8 bits)
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
    currentState = EEPROM.read(0);
    SerialPrintDebug("CurrentState of POP_UP_BUOY: ");
    SerialPrintDebugln(currentState);


    currentState = 2;

  //------- PB DEFINITION ----------------------------------------------------------------------------------
    pinMode(PB_1, INPUT_PULLUP);
    pinMode(PB_2, INPUT_PULLUP);
    pinMode(PB_3, INPUT_PULLUP);

  //------- LED DEFINITION ---------------------------------------------------------------------------------
    pinMode(LED_R, OUTPUT);
    pinMode(LED_Y, OUTPUT);
    pinMode(LED_G, OUTPUT);

    digitalWrite(LED_Y, HIGH);  // turn on led to know that the board is in setup mode
    digitalWrite(LED_R, LOW);   // initialise off
    digitalWrite(LED_G, HIGH);  // initialise off

  //------- POWER RELAY DEFINITION -------------------------------------------------------------------------
    pinMode(GPS_KIM, OUTPUT);
    pinMode(SD_card, OUTPUT);
    digitalWrite(SD_card, HIGH);
    if (currentState == 4 or currentState == 5 or currentState == 6) {
      digitalWrite(GPS_KIM, HIGH);  
    } else{
      digitalWrite(GPS_KIM, LOW); 
    }

  //------- EXTERNAL RTC SETUP---------------------------------------------------------------------------
    if (currentState == 0 or currentState == 1 or currentState == 2 or currentState == 3 or currentState == 4 or currentState == 5 or currentState == 6) {
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

      // Making it so, that the alarm will trigger an interrupt
      pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

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
    if (currentState == 0) {
      SerialPrintDebugln("RTC Configuration. Push button 1 for NTP sync. (default) or 2 for Lander sync.");
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
          SerialPrintDebugln("ERROR -- button 3 pushed. Changing to state 1");
          changeStateTo(1);//change to state 1
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

  //------- TIME UPDATE FROM UDP SERVER OR LANDER --------------------------------------------------------------------
      char date[10] = "hh:mm:ss";
      rtcExt.now().toString(date);
      writeLogFile("The time on the RTC is " + String(date));
          
    if (currentState == 0 && PBState == 1 && WiFi.status() == WL_CONNECTED) {
      //sincronise time from NTP server
      SerialPrintDebugln("Obtaining time from NTP server");
      timeClient.begin();
      timeClient.update();
      rtcExt.adjust(DateTime(timeClient.getEpochTime()));
      
      rtcExt.now().toString(date);
      SerialPrintDebug("time changed in RTC DS3231. current time:");
      SerialPrintDebugln(date);

    } else if (currentState == 0 && PBState == 2 && WiFi.status() == WL_CONNECTED){
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
        writeLogFile("Adjustment of SYNCTIME failed for wrong HTTP request!" );
      }else{
        // Ajustar el SYNCTIME con los valores obtenidos
        EEPROM.write(6, syncTime);
        EEPROM.commit();
        delay(50);
        writeLogFile("Adjustment of SYNCTIME of buoy " +String(idBuoy)+ " done." );
      }
    }

  //------- SD CARD SETUP ----------------------------------------------------------------------------------
    delay(100);
    if (currentState == 0 or currentState == 1 or currentState == 2 or currentState == 3 or currentState == 4 or currentState == 5 or currentState == 6) {
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
    if (currentState == 0 or currentState == 1 or currentState == 2 or currentState == 3 or currentState == 4 or currentState == 5 or currentState == 6) {      
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
    }
  //------- SLEEP MODE SETUP -------------------------------------------------------------------------------
    // Two ways of waking up, by a timmer or by an external iterruption
    //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // pin for the external RTC

  //------- GPS MODULE SETUP -------------------------------------------------------------------------------
    if (currentState == 4 or currentState == 5 or currentState == 6) {
      SerialPrintDebugln("GPS Module Setup ---->");
      gpsSerial.begin(GPSBaud);
      SerialPrintDebug(F("Testing TinyGPSPlus library v. "));
      SerialPrintDebugln(TinyGPSPlus::libraryVersion());
      delay(10);
      SerialPrintDebugln("GPS Module Setup ----> DONE");
    }
  //------- KIM MODULE SETUP -------------------------------------------------------------------------------
    if (currentState == 4 or currentState == 5 or currentState == 6) {
      SerialPrintDebugln("KIM Module Setup ---->");
      while (!KIM.check()) {
        SerialPrintDebugln("Failed connexion to KIM module. Retriying in 3s...");
        delay(1000);
      }
      SerialPrintDebugln(KIM.get_SN());
      delay(delayKIM);
      SerialPrintDebugln(KIM.get_ID());
      delay(delayKIM);
      SerialPrintDebugln(KIM.get_PWR());
      delay(delayKIM);
      SerialPrintDebugln(KIM.get_AFMT());
      delay(delayKIM);
      SerialPrintDebugln("KIM Module Setup ----> DONE");
    }
 //------- ADC SETUP ---------------------------------------------------------------------------------------
  if (currentState == 4 or currentState == 5 or currentState == 6) {
    SerialPrintDebugln("ADC Setup ---->");
    analogReadResolution(ADC_resolution);
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

    Serial.println("loop init");

    Serial.println("Connect to wifi");
      if (!connectToRaspWiFi()) {
        Serial.println("Coul not connect wo wifi!!!!");
        while (1) {}
      }

      if (connectToFTP()) {
        Serial.println("connectToFTP failed!!!");
        while (1) {}
      }
      Serial.print("FTP initialized");

      bool test_permission = false;
      Serial.println("requesting permission");
      while (!test_permission) {
        test_permission = requestTestStartPermission();
        if (!test_permission) {
          Serial.println("No permission for testing yet");
          delay(5000);
        }
      }
      Serial.println("Permissiong granted, asking for distance");
      String test_distance = getTestDistance();
      startDistanceTest();
      writeLogFile("== START DISTANCE TEST WITH " + String(test_distance)  + " mm");
      idBuoy = 1;
      downloadAllFilesFTP();
      idBuoy = 2;
      downloadAllFilesFTP();
      idBuoy = 3;
      downloadAllFilesFTP();
      Serial.println("Test ended, sending http for end");
      endDistanceTest();
      Serial.println("all done");
      deleteFilesInFolder("/PopUpBuoy_1");
      deleteFilesInFolder("/PopUpBuoy_2");
      deleteFilesInFolder("/PopUpBuoy_3");
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
bool writeLogFile(String message) {

  struct tm timeinfo;

  message = "State " + String(currentState) + " - " + message;

  SerialPrintDebug("Writing in LogFile.txt ---");
  SerialPrintDebugln(message);
  
  //Open file and create if it doeesn't exist
  LogFile = SD.open(Log_filename, FILE_APPEND);  //filename is the file name to be created and FILE_WRITE is a command to create file.
  if (!LogFile) {
    LogFile.close();  //Closing the file
    return false;
  }

  DateTime timeRtcExt = rtcExt.now();  //To have the UTC time and not the local one
  if (!LogFile.print(String(timeRtcExt.timestamp(DateTime::TIMESTAMP_FULL)))) {
    return false;
  }

  if (!LogFile.print("----")) {
    return false;
  }


  if (!LogFile.println(message)) {
    return false;
  }

  //close file
  LogFile.close();  //Closing the file

  return true;
}
void SerialPrintDebug(int message){
  #ifdef SERIAL_DEBUG
    Serial.print(message);
  #endif
}
void SerialPrintDebugln(int message){
  #ifdef SERIAL_DEBUG
    Serial.println(message);
  #endif
}
void SerialPrintDebug(String message){
  #ifdef SERIAL_DEBUG
    Serial.print(message);
  #endif
}
void SerialPrintDebugln(String message){
  #ifdef SERIAL_DEBUG
    Serial.println(message);
  #endif
}
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
void eepromSaveState(int newstate) {
  EEPROM.write(0, newstate);
  EEPROM.commit();
  delay(10);
}
void eepromInitState() {
  EEPROM.write(0, 0);
  EEPROM.commit();
  delay(10);
  currentState = EEPROM.read(0);
}
void changeStateTo(int newState) {
  currentState = newState;
  eepromSaveState(currentState);
}
void configureKIM(){
  SerialPrintDebugln("KIM Initial Setup ---->");
  while (!KIM.check()) {
    SerialPrintDebugln("Failed connexion to KIM module. Retriying in 3s...");
    delay(1000);
  }
  if(currentState == 0 or currentState == 1 or currentState == 2 or currentState == 3 or currentState == 4 or currentState == 5){
    KIM.set_PWR(PWR2, strlen(PWR2));  // AT+PWR=1  & AT+AFMT=1  &  AT+SAVE_CFG  --> These three comands should be sent once and they will be kept on RAM (New default).
    delay(delayKIM); 
    writeLogFile("KIM power changed to: " + String(KIM.get_PWR()));
  }else{
    KIM.set_PWR(PWR3, strlen(PWR3));  // AT+PWR=1  & AT+AFMT=1  &  AT+SAVE_CFG  --> These three comands should be sent once and they will be kept on RAM (New default).
    delay(delayKIM);  
    writeLogFile("KIM power changed to: " + String(KIM.get_PWR()));
  }
  delay(delayKIM);                    // IMPORTANT because by default AT+AFMT=0 and then it sends RAW messages
  KIM.set_AFMT(AFMT, sizeof(AFMT) - 1); 
  delay(delayKIM); 
  if (KIM.save_CFG() == OK_KIM) {
    writeLogFile("Kim Configuration_OK");
  } else {
    writeLogFile("Kim Configuration_ERR");
  }
  delay(delayKIM); 
}

//------- FUNCTIONS FOR SLEEP SEQUENCE ---------------------------------------------------------------------
void SleepModeSequence(int8_t sleepingHours, int8_t sleepingMinute, int8_t sleepingSecond, int sleepMode) {
  //Disconnect Peripherals
  ConnectPeripherals(false, GPS_KIM);
  delay(10);
  ConnectPeripherals(false, SD_card);
  //Light Sequence
  lightSequenceSleep();
  //Enter Sleep mode
  if(sleepMode == 0){
    SerialPrintDebugln("Sleeping relative time");
    goToSleepRTC_rel(sleepingHours, sleepingMinute, sleepingSecond);
  }else{
    SerialPrintDebugln("Sleeping absolute time");
    goToSleepRTC_abs(sleepingHours);
  }
    
}
void goToSleep(int sleeping_time) {  //no need to turn off pheriperals, already done
  SerialPrintDebugln("Starting Light Sleep Routine");
  delay(10);
  if (sleeping_time == 0){
    sleeping_time=1;
  }
  //SD.end();
  esp_sleep_enable_timer_wakeup(sleeping_time * uS_TO_S_FACTOR);
  esp_light_sleep_start(); 
}
void goToSleepRTC_rel(int8_t sleepingHours, int8_t sleepingMinute, int8_t sleepingSecond) {

   rtcExt.clearAlarm(1);
  if (!rtcExt.setAlarm1(rtcExt.now() + TimeSpan(0, sleepingHours, sleepingMinute, sleepingSecond), DS3231_A1_Hour)) {
    SerialPrintDebugln("Error, alarm wasn't set!");

  } else {
    //SerialPrintDebug("Alarm will happen in 10 seconds!");
    DateTime now = rtcExt.now();

    // the stored alarm value + mode
    DateTime alarm1 = rtcExt.getAlarm1();
    Ds3231Alarm1Mode alarm1mode = rtcExt.getAlarm1Mode();
    char alarm1Date[20] = "hh:mm:ss";
    alarm1.toString(alarm1Date);
    SerialPrintDebug(" [Alarm1: ");
    SerialPrintDebug(alarm1Date);
    SerialPrintDebug(", Mode: ");
    switch (alarm1mode) {
      case DS3231_A1_PerSecond: SerialPrintDebugln("PerSecond"); break;
      case DS3231_A1_Second: SerialPrintDebugln("Second"); break;
      case DS3231_A1_Minute: SerialPrintDebugln("Minute"); break;
      case DS3231_A1_Hour: SerialPrintDebugln("Hour"); break;
      case DS3231_A1_Date: SerialPrintDebugln("Date"); break;
      case DS3231_A1_Day: SerialPrintDebugln("Day"); break;
    }
  }
  SerialPrintDebugln("going to sleep");
  //SD.end();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // pin for the external RTC
  esp_deep_sleep_start();
}
void goToSleepRTC_abs(int8_t sleepingHours) {
  rtcExt.clearAlarm(1);

  DateTime now = rtcExt.now();
  DateTime alarmTime;
  syncTime = EEPROM.read(6);

  if (sleepingHours == 24) {
    // Configurar la alarma para las 9:00 AM de hoy si aún no ha pasado; si no, configurar para las 9:00 AM del día siguiente
    if (now.hour() < syncTime) {
      alarmTime = DateTime(now.year(), now.month(), now.day(), syncTime, 0, 0);
    } else {
      alarmTime = DateTime(now.year(), now.month(), now.day(), syncTime, 0, 0) + TimeSpan(1, 0, 0, 0);
    }
  } else {
    // Configurar la alarma para la próxima hora en punto transcurridas las sleepingHours
    alarmTime = DateTime(now.year(), now.month(), now.day(), now.hour(), 0, 0) + TimeSpan(0, sleepingHours, 0, 0);
  }

  if (!rtcExt.setAlarm1(alarmTime, DS3231_A1_Date)) {
    SerialPrintDebugln("Error, alarm wasn't set!");
  } else {
    // Imprimir fecha y hora de la alarma en la misma línea que el modo
    char alarm1Date[20] = "YYYY-MM-DD hh:mm:ss";
    alarmTime.toString(alarm1Date); 
    SerialPrintDebug(" [Alarm1: ");
    SerialPrintDebug(alarm1Date);  // Imprimir fecha y hora en la misma línea
    SerialPrintDebugln(", Mode: Date");
  }
  SerialPrintDebugln("going to sleep");
  //SD.end();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // pin para la alarma del RTC
  esp_deep_sleep_start();
}
void onAlarm() {
  SerialPrintDebugln("Alarm occured!");
}
void ConnectPeripherals(bool activateRelay, int PRelay) {
  #ifdef DISCONNECT_PHER
    if (activateRelay == true) {
      SerialPrintDebugln("Activating Peripherals in " + String(PRelay));
      digitalWrite(PRelay, HIGH);

    } else {
      SerialPrintDebugln("Deactivating Peripherals in " + String(PRelay));
      digitalWrite(PRelay, LOW);
    }
  #endif
}
void lightSequenceSleep() {
  //slow flash yellow led
  for (int i = 0; i <= 2; i++) {
    digitalWrite(LED_Y, HIGH);
    delay(500);
    digitalWrite(LED_Y, LOW);
    delay(500);
  }

  //fast flash yellow led
  for (int i = 0; i <= 5; i++) {
    digitalWrite(LED_Y, HIGH);
    delay(100);
    digitalWrite(LED_Y, LOW);
    delay(100);
  }
}
bool parseTimeResponse(const String &payload, int &year, int &month, int &day, int &hour, int &minute, int &second) {
    // Verificar si el campo "success" es true
    if (payload.indexOf("\"success\": true") == -1) {
        writeLogFile("Request failed: Time parsing failed due to missing success flag.");
        return false;
    }

    // Extraer el año
    int yearStart = payload.indexOf("\"year\":");
    if (yearStart != -1) {
        yearStart = payload.indexOf(":", yearStart) + 1;
        year = payload.substring(yearStart, payload.indexOf(",", yearStart)).toInt();
        //SerialPrintDebugln("Year is: " + String(year));
    } else {
        writeLogFile("Missing year in response");
        return false;
    }

    // Extraer el mes
    int monthStart = payload.indexOf("\"month\":");
    if (monthStart != -1) {
        monthStart = payload.indexOf(":", monthStart) + 1;
        month = payload.substring(monthStart, payload.indexOf(",", monthStart)).toInt();
        //SerialPrintDebugln("Month is: " + String(month));
    } else {
        writeLogFile("Missing month in response");
        return false;
    }

    // Extraer el día
    int dayStart = payload.indexOf("\"day\":");
    if (dayStart != -1) {
        dayStart = payload.indexOf(":", dayStart) + 1;
        day = payload.substring(dayStart, payload.indexOf(",", dayStart)).toInt();
        //SerialPrintDebugln("Day is: " + String(day));
    } else {
        writeLogFile("Missing day in response");
        return false;
    }

    // Extraer la hora
    int hourStart = payload.indexOf("\"hour\":");
    if (hourStart != -1) {
        hourStart = payload.indexOf(":", hourStart) + 1;
        hour = payload.substring(hourStart, payload.indexOf(",", hourStart)).toInt();
        //SerialPrintDebugln("Hour is: " + String(hour));
    } else {
        writeLogFile("Missing hour in response");
        return false;
    }

    // Extraer el minuto
    int minuteStart = payload.indexOf("\"minute\":");
    if (minuteStart != -1) {
        minuteStart = payload.indexOf(":", minuteStart) + 1;
        minute = payload.substring(minuteStart, payload.indexOf(",", minuteStart)).toInt();
        //SerialPrintDebugln("Minute is: " + String(minute));
    } else {
        writeLogFile("Missing minute in response");
        return false;
    }

    // Extraer el segundo
    int secondStart = payload.indexOf("\"second\":");
    if (secondStart != -1) {
        secondStart = payload.indexOf(":", secondStart) + 1;
        second = payload.substring(secondStart, payload.indexOf("}", secondStart)).toInt();
        //SerialPrintDebugln("Second is: " + String(second));
    } else {
        writeLogFile("Missing second in response");
        return false;
    }

    // Log de éxito
    String log = "Parsed time response successfully. ";
    log += "Year: " + String(year) + ", ";
    log += "Month: " + String(month) + ", ";
    log += "Day: " + String(day) + ", ";
    log += "Hour: " + String(hour) + ", ";
    log += "Minute: " + String(minute) + ", ";
    log += "Second: " + String(second);
    
    writeLogFile(log);
    return true;
}
bool parsePermissionResponse(const String &payload, int &releaseFlag, ReleaseMode &releaseMode, int &sleeptime_h, int &sleeptime_m) {
    // Verificar si el campo "success" es true
    if (payload.indexOf("\"success\": true") == -1) {
        // Extraer el mensaje de error si "success" no es true
        int messageStart = payload.indexOf("\"message\":\"");
        if (messageStart != -1) {
            messageStart += 10;  // Ajustar posición al valor del mensaje
            int messageEnd = payload.indexOf("\"", messageStart);
            String message = payload.substring(messageStart, messageEnd);
            String log = "Request failed: " + message;
            writeLogFile(log);
        } else {
            writeLogFile("Request failed: Unknown error");
        }
        return false;
    }

    // Extraer releaseFlag (manejar espacios antes del valor)
    int flagStart = payload.indexOf("\"releaseFlag\":");
    if (flagStart != -1) {
        flagStart = payload.indexOf(":", flagStart) + 1;  // Ajustar posición al valor
        releaseFlag = payload.substring(flagStart, payload.indexOf(",", flagStart)).toInt();
        //SerialPrintDebugln("releaseFlag is " + String(releaseFlag));
    } else {
        writeLogFile("Missing releaseFlag in response");
        return false;
    }

    // Extraer releaseMode
    int modeStart = payload.indexOf("\"releaseMode\": ");
    if (modeStart != -1) {
        modeStart = payload.indexOf(":", modeStart) + 2;  // Ajustar posición al valor, +2 para incluir el siguiente "
        int modeEnd = payload.indexOf(",", modeStart);
        String releaseModeStr = payload.substring(modeStart, modeEnd); // Sin trim()
        //SerialPrintDebugln("releaseModeStr is: " + releaseModeStr);
        if (releaseModeStr == "\"FRM\"") {
            releaseMode = FRM;
        } else if (releaseModeStr == "\"DM\"") {
            releaseMode = DM;
        } else {
            writeLogFile("Unknown releaseMode: " + releaseModeStr + ". Forcing releaseMode to DM.");
            releaseMode = DM;
        }
    } else {
        writeLogFile("Missing releaseMode in response");
        return false;
    }

    // Extraer sleeptime_h (manejar espacios antes del valor)
    int sleepHStart = payload.indexOf("\"sleeptime_h\":");
    if (sleepHStart != -1) {
        sleepHStart = payload.indexOf(":", sleepHStart) + 3;  // Ajustar posición al valor
        String sleeptime_h_str = payload.substring(sleepHStart, payload.indexOf(",", sleepHStart)-1);
        //SerialPrintDebugln("sleeptime_h is: " + sleeptime_h_str);
        sleeptime_h =  sleeptime_h_str.toInt();
    } else {
        writeLogFile("Missing sleeptime_h in response");
        return false;
    }

    // Extraer sleeptime_m (manejar espacios antes del valor)
    int sleepMStart = payload.indexOf("\"sleeptime_m\":");
    if (sleepMStart != -1) {
        sleepMStart = payload.indexOf(":", sleepMStart) + 3;  // Ajustar posición al valor
        String sleeptime_m_str = payload.substring(sleepMStart, payload.indexOf("}", sleepMStart)-1);
        //SerialPrintDebugln("sleeptime_m is: " + sleeptime_m_str);
        sleeptime_m = sleeptime_m_str.toInt();
    } else {
        writeLogFile("Missing sleeptime_m in response");
        return false;
    }

    // Log para indicar éxito con los valores extraídos
    String log = "Parsed permission response successfully. ";
    log += "releaseFlag: " + String(releaseFlag) + ", ";
    log += "releaseMode: " + String(releaseMode == FRM ? "FRM" : "DM") + ", ";
    log += "sleeptime_h: " + String(sleeptime_h) + ", ";
    log += "sleeptime_m: " + String(sleeptime_m);
    
    writeLogFile(log);
    return true;
}
bool parseSyncTimeResponse(const String& payload) {
    // Verificar si el campo "sync_time" está presente
    int syncTimeStart = payload.indexOf("\"sync_time\":");
    if (syncTimeStart != -1) {
        syncTimeStart += 12;  // Ajustar posición al valor
        syncTime = payload.substring(syncTimeStart, payload.indexOf("}", syncTimeStart)).toInt();
        SerialPrintDebugln("syncTime is " + String(syncTime));
        writeLogFile("Parsed syncTime successfully. syncTime: " + String(syncTime));
        return true;
    } else {
        writeLogFile("Missing syncTime in response");
        return false;
    }
}

// Get Test Distacne //
String getTestDistance(void){
  HTTPClient http;
  String response;
  String action_string;
  String url;
  url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + "/" + "distance_get_mm";
  String distance = "";
  http.begin(url);

  // Realizar la solicitud GET
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {  
    response = "HTTP response successful: " + String(httpResponseCode)+ " " + http.getString();
    String json = http.getString();
    int startIndex = json.indexOf("\"distance\":");
    if (startIndex == -1) {
      Serial.println("Key not found.");
      return distance;
    }

    // Move to the value start (after the colon and space)
    startIndex = json.indexOf("\"", startIndex + 10) + 1;
    int endIndex = json.indexOf("\"", startIndex);

    // Extract the distance value
    distance = json.substring(startIndex, endIndex);
  
  } else {
    response = "HTTP error while getting test distance: " + String(httpResponseCode)+ " " + http.getString();
    writeLogFile(response);
  }
  http.end();
  return distance;
}

// Get Test Distacne //
void startDistanceTest(void){
  HTTPClient http;
  String response;
  String action_string;
  String url;
  url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + "/" + "init_distance_test";
  http.begin(url);
  int httpResponseCode = http.GET();
  writeLogFile("HTTP request to begin distance test, response: " + String(httpResponseCode));
  http.end();
}

// Get Test Distacne //
void endDistanceTest(void){
  HTTPClient http;
  String response;
  String action_string;
  String url;
  url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + "/" + "end_distance_test";
  http.begin(url);
  int httpResponseCode = http.GET();
  writeLogFile("HTTP request to end distance test, response: " + String(httpResponseCode));
  http.end();
}

//Request test start permissions //
bool getPermissionValue(String json) {
  int index = json.indexOf("\"permission\":");
  if (index == -1) {
    // Key not found; default or error handling
    return false;
  }

  // Find the start of the value (true/false)
  index = json.indexOf(":", index) + 1;
  String value = json.substring(index);
  value.trim();

  return value.startsWith("true");
}

// Get Test Distance //
bool requestTestStartPermission(void){
  HTTPClient http;
  String response;
  String action_string;
  String url;
  url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + "/" + "init_distance_test_permission";
  http.begin(url);

  bool permission = false;

  // Realizar la solicitud GET
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {  
    String payload = http.getString();
    
    permission = getPermissionValue(payload);
     
  } else {
    response = "HTTP error while getting test distance: " + String(httpResponseCode)+ " " + http.getString();
    writeLogFile(response);    
  }
  http.end();
  return permission;
}


bool sendHttpGetRequest(int idBoia, ActionType action, int &releaseFlag, ReleaseMode &releaseMode, int &sleeptime_h, int &sleeptime_m) {
  HTTPClient http;
  String response;
  String action_string;
  String url;

  switch (action) {
    case RELEASE:
      action_string = "/release/";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string + String(idBoia);
      break;
    case PERMISSION:
      action_string = "/permission/";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string + String(idBoia);
      break;
    case GETTIME:
      action_string = "/gettime";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string;
      break; 
    case GETSYNCTIME:
      action_string = "/getsynctime";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string;
      break;           
    default:
      action_string = "/unknown/";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string + String(idBoia);
      break;
  }


  // Comenzar la conexión HTTP
  http.begin(url);

  // Realizar la solicitud GET
  int httpResponseCode = http.GET();

  switch (action){
    case RELEASE:
      // Verificar el código de respuesta
      if (httpResponseCode == 200) {
        response = "HTTP response successful: " + String(httpResponseCode)+ " " + http.getString();
        writeLogFile(response);
        http.end();
        return true;
      } else {
        response = "Error in the HTTP request: " + String(httpResponseCode)+ " " + http.getString();
        writeLogFile(response);
        http.end();
        return false;
      }
      break;
    case PERMISSION:
      if (httpResponseCode == 200) {
        String payload = http.getString();  // Obtén la respuesta como string
 
        // Llamar a la subfunción para parsear el payload
        bool parsepermissionSuccess = parsePermissionResponse(payload, releaseFlag, releaseMode, sleeptime_h, sleeptime_m);
        //writeLogFile("Message parsed. releaseFlag = " + String(releaseFlag) + ", releaseMode = " + String (releaseMode) + ", sleeptime_h = " + String(sleeptime_h) + " and sleeptime_m = " + String(sleeptime_m));

        http.end();  // Liberar recursos
        return parsepermissionSuccess;
      } else {
        // Si el código de respuesta no es 200
        response = "Error in the HTTP request: "+ String(httpResponseCode)+ " "  + http.getString();
        writeLogFile(response);
        http.end();  // Liberar recursos
        return false;
      }
      break;
    case GETTIME:
      if (httpResponseCode == 200) {
        String payload = http.getString();  // Obtén la respuesta como string
        int year, month, day, hour, minute, second;
        // Llamar a la subfunción para parsear el payload
        bool parsetimeSuccess = parseTimeResponse(payload, year, month, day, hour, minute, second);
        //{"success": true, "current_time": {"year": 2024, "month": 10, "day": 10, "hour": 10, "minute": 40, "second": 34}}
        year_lander=year;
        month_lander=month;
        day_lander=day;
        hour_lander=hour;
        minute_lander=minute;
        second_lander=second;
        http.end();  // Liberar recursos
        return parsetimeSuccess;
      } else {
        // Si el código de respuesta no es 200
        response = "Error in the HTTP request: "+ String(httpResponseCode)+ " "  + http.getString();
        writeLogFile(response);
        http.end();  // Liberar recursos
        return false;
      }
      break;  
    case GETSYNCTIME:
      if (httpResponseCode == 200) {
        String payload = http.getString();
        if (!parseSyncTimeResponse(payload)) {
          writeLogFile("Failed to parse sync_time");
          return false;
        }
        return true;
      } else {
        writeLogFile("HTTP GET failed for GETSYNCTIME");
        return false;
      }
      break; 
    default:
      response = "Unknown action requested: " + String(action) + " - HTTP response code: " + String(httpResponseCode);
      writeLogFile(response);
      return false;
      break;
  }

  // Liberar recursos
  http.end();
  return false;
}
bool connectToRaspWiFi() {
  WiFi.begin(WIFI_SSID2, WIFI_PASS2);
  unsigned long startAttemptTime = millis();

  SerialPrintDebug("Conectando a WiFi...");
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < maxWIFITimeout) {
    delay(500);
    SerialPrintDebug(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    SerialPrintDebugln("Connected!");
    writeLogFile("Connected to Pop-Up server WiFi");
    return true;
  } else {
    SerialPrintDebugln("Not connected!");
    String response = "ERROR: Impossible to connect to Pop-Up server WiFi. Reason: " + getWiFiFailureReason(WiFi.status());
    writeLogFile(response);
    return false;
  }
}
String getWiFiFailureReason(int status) {
  switch (status) {
    case WL_IDLE_STATUS: return "Idle status";
    case WL_NO_SSID_AVAIL: return "No SSID available";
    case WL_SCAN_COMPLETED: return "Scan completed";
    case WL_CONNECT_FAILED: return "Connection failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED: return "Disconnected";
    default: return "Unknown";
  }
}
//------- FUNCTIONS FOR SURFACE SEQUENCE ------------------------------------------------------------------
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


  SerialPrintDebugln("Saving GPS data to SD ---> DONE");
}
void adcAcquireData(char *ADCreadHex) {

  float R1_ADC = 10; // Resistance R1 value in MΩ
  float R2_ADC = 10; // Resistance R2 value in MΩ
  float Vref_ADC = 3.3; // Tensión de referencia del ADC (3.3V)
  float a_calADC = 1; //0.9637; // calibration as y=ax+b
  float b_calADC = 0; //0.500; 
  float Vout_ADC;
  int ADCread;
  int sumaADCread = 0;

  for (int i = 0; i < 100; ++i) {
    int ADCread = analogRead(ADC_PIN);  // Realiza la lectura analógica
    sumaADCread += ADCread;  // Suma la lectura actual a la suma total
    delay(10);  // Espera antes de la próxima lectura
  }

  ADCread = sumaADCread/100; // Realizar la lectura analógica

  // Calcular el voltaje antes del divisor de tensión basado en la lectura del ADC y la referencia de voltaje
  Vout_ADC = (ADCread*Vref_ADC) /(pow(2, ADC_resolution) - 1);

  // Calcular el valor en voltaje basado en la lectura del ADC y el divisor de tensión + calibración
  Vin_ADC = Vout_ADC * ((R1_ADC + R2_ADC) / R2_ADC)*a_calADC+b_calADC;

  // Convertir la lectura de 8 bits a hexadecimal (2 caracteres) y mostrarlo
  sprintf(ADCreadHex, "%02X", ADCread); // Convertir a hexadecimal
  delay(100);
  
  // Mostrar la lectura del ADC en 8 bits
  SerialPrintDebug("ADC read (8 bits): ");
  SerialPrintDebugln(ADCread);
  // Mostrar el valor de voltaje calculado después del divisor de tensión
  SerialPrintDebug("Vout (in the ADC): ");
  char buffer[10];
  dtostrf(Vout_ADC, 6, 5, buffer);
  SerialPrintDebug(buffer);
  SerialPrintDebugln(" V");
  // Mostrar el voltaje antes del divisor de tensión
  SerialPrintDebug("Vin (up): ");
  dtostrf(Vin_ADC, 6, 5, buffer);
  SerialPrintDebug(buffer);
  SerialPrintDebugln(" V");
}
bool sendGPSviaKIM(int sendRepeat, int waitRepeat) {

  for (int i = 0; i < sendRepeat; i++) {
    currentState = EEPROM.read(0);
    if (currentState!=6){
      ConnectPeripherals(true, GPS_KIM);  // turn on power to all devices
    }
    //update gps and mask it again
    delay(10);
    if (KIM.send_data(kineisMessage, sizeof(kineisMessage) - 1) == OK_KIM) {
      delay(INTERVAL_SEND_MS);
      writeLogFile(" Kim MSG_OK");
    } else {
      writeLogFile(" Kim MSG_ERR");
    }

    if (currentState!=6){
      ConnectPeripherals(false, GPS_KIM);  // turn off power to all devices (not in case &)
    }
    
    delay(10);
    goToSleep((waitRepeat-INTERVAL_SEND_MS)/1000);
  }
  return true;
}
void maskGPS(double &gpsLat, double &gpsLong, uint32_t &epochTime, char *kineisMessage, char *ADCreadHex) {
  // this function masks the gps latitute and longitude in hexadecimal
  int latitude, longitude;
  char maskedData[25];
  char hex_latitude[9], hex_longitude[9];
  char hex_epochTime[9];
  char hex_crc[3];

  if (gpsLat == 200 && gpsLong == 200){  //means GPS is not fixed

    strcpy(hex_latitude, "FFFFFFFF");
    strcpy(hex_longitude, "FFFFFFFF");

  } else {

    latitude = gpsLat * (pow(10, 6));    //multiply 10^6 to eliminate decimals
    longitude = gpsLong * (pow(10, 6));  //multiply to eliminate decimals

    if (latitude < 0) {  // check if value is positive or negative
      sprintf(hex_latitude, "%08lX", (unsigned long)(4294967296 + latitude));
    } else {
      sprintf(hex_latitude, "%08lX", (unsigned long)latitude);
   }

    if (longitude < 0) {  // check if value is positive or negative
      sprintf(hex_longitude, "%08lX", (unsigned long)(4294967296 + longitude));
    } else {
      sprintf(hex_longitude, "%08lX", (unsigned long)longitude);
    }

  }

  sprintf(hex_epochTime, "%08lX", (unsigned long)epochTime);

  //append all the informations
  sprintf(maskedData, "%s%s%s", hex_latitude, hex_longitude, hex_epochTime);

  #ifdef WORK_ADC
    sprintf(kineisMessage, "%s%s", maskedData, ADCreadHex);
  #else
    //calculate CRC8
    FastCRC8 CRC8;
    uint8_t crc = CRC8.smbus((uint8_t *)maskedData, 24);
    sprintf(hex_crc, "%02lX", (unsigned long)crc);
    sprintf(kineisMessage, "%s%s", maskedData, hex_crc);
  #endif

  writeLogFile("Message to transmitt kineisMessage: ");
  writeLogFile(kineisMessage);

  // Reinicia el buffer para la próxima conversión
  memset(hex_longitude, 0, sizeof(hex_longitude));
  memset(hex_latitude, 0, sizeof(hex_latitude));
  memset(hex_epochTime, 0, sizeof(hex_epochTime));
}
float updateMinElev() {
  // Llamada para leer el progreso actual desde el archivo
  readSuccessFile();
  //writeLogFile("RowProgress = " + String(RowProgress));
  
  // Actualización de MinElev según el valor de RowProgress
  if (RowProgress >= 1 && RowProgress <= 993) {
      return 35.0;
  } else if (RowProgress >= 994 && RowProgress <= 1986) {
      return 30.0;
  } else if (RowProgress >= 1987 && RowProgress <= 2979) {
      return 25.0;
  } else if (RowProgress >= 2980 && RowProgress <= 3972) {
      return 20.0;
  } else if (RowProgress >= 3973 && RowProgress <= 4965) {
      return 15.0;
  } else if (RowProgress >= 4966) {
      return 10.0;
  } else {
      // Valor por defecto si RowProgress no es válido
      return 10.0;
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

  //MinElev = updateMinElev();
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
    30                                                                //< Computation step (default 30s)
  };

  struct SatelliteNextPassPrediction_t nextPass;
  struct SatelliteNextPassPrediction_t earliestPass;

  for (uint8_t i = 0; i < nbSatsInAopTable; i++) {
    PREVIPASS_compute_next_pass(&prepasConfiguration, &aopTable[i], 1, &nextPass);
    if (i == 0 || nextPass.epoch < earliestPass.epoch) {  // Comparison with time now so it won't bring back a SPP already begun
      earliestPass = nextPass;
      delay(100);
    }
  }
  //messageLogFile = "For the SPP : Next satellite epoch : " + String(earliestPass.epoch) + " and epoch now : " + String(now.unixtime());
  //writeLogFile(messageLogFile);

  //! Sat name
  char satNameTwoChars[3];

  switch (earliestPass.satHexId) {
    case 0x2:
      strcpy(satNameTwoChars, "03");
      break;
    case 0x6:
      strcpy(satNameTwoChars, "A1");
      break;
    case 0xA:
      strcpy(satNameTwoChars, "MA");
      break;
    case 0x9:
      strcpy(satNameTwoChars, "MB");
      break;
    case 0xB:
      strcpy(satNameTwoChars, "MC");
      break;
    case 0x5:
      strcpy(satNameTwoChars, "NK");
      break;
    case 0x8:
      strcpy(satNameTwoChars, "NN");
      break;
    case 0xC:
      strcpy(satNameTwoChars, "NP");
      break;
    case 0xD:
      strcpy(satNameTwoChars, "SR");
      break;
    default:
      strcpy(satNameTwoChars, "XX");
  }

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
  Decimal_CoverageDuration = (EEPROM.read(2) << 8) | EEPROM.read(3);
  SerialPrintDebugln(" Time of coverage next satellite : " + String(Decimal_CoverageDuration));

  messageLogFile = "Next Satelitte : Time before next satellite :" + String(diff) + " sec and coverage : " + String(Decimal_CoverageDuration) + String(" sec");
  writeLogFile(messageLogFile);

  return diff;
}
void initializeEEPROM() {
    int storedValue = EEPROM.read(0);
    
    if (storedValue == 0xFF) { // Si la EEPROM no ha sido inicializada
        writeLogFile("EEPROM no ini., saving default value.");
        EEPROM.write(0, 0);
        EEPROM.commit();
        storedValue = 6;
    }
    
    currentState = storedValue;
}
void eepromSaveTimeCoverage(int timeCoverage) {
  EEPROM.write(2, (timeCoverage >> 8) & 0xFF);
  EEPROM.write(3, timeCoverage & 0xFF);
  EEPROM.commit();
  delay(50);
}
void SetCoverageDurationTo_0() {
  Decimal_CoverageDuration = 0;
  eepromSaveTimeCoverage(Decimal_CoverageDuration);
}
void SetCoverageStateTo(int NewCoverageState) {
  EEPROM.write(1, NewCoverageState);
  EEPROM.commit();
  delay(50);
}
void eepromSaveCounterGPSFail(int counter) {
  EEPROM.write(4, counter);
  EEPROM.commit();
  delay(50);
}
void SetCounterFailGPSTo_0() {
  Counter_FailGPS = 0;
  eepromSaveCounterGPSFail(Counter_FailGPS);
}
void eepromSaveCounterWIFIFail(int counter) {
  EEPROM.write(5, counter);
  EEPROM.commit();
  delay(50);
}
void SetCounterFailWIFITo_0() {
  Counter_FailWIFI = 0;
  eepromSaveCounterWIFIFail(Counter_FailWIFI);
}
void IncrementCounterFailWIFI(){
  if(Counter_FailWIFI >= 2){ //Means needs to start from 0
    Counter_FailWIFI = 0;
  }else{
    Counter_FailWIFI = Counter_FailWIFI + 1;
  }
  eepromSaveCounterWIFIFail(Counter_FailWIFI);
}
void ChangeSecondsInHoursAndMinutes(int *seconds, int *minutes, int *hours) {
  *hours = *seconds / 3600;           // Conversion en heures
  *minutes = (*seconds % 3600) / 60;  // Conversion en minutes
  *seconds = (*seconds % 3600) % 60;  // Conversion en secondes sans les heures et les minutes
}
void SendGPSMessage(int timeSending) {

  maskGPS(gpsLat, gpsLong, epochTime, kineisMessage, ADCreadHex);  
  sendGPSviaKIM(timeSending / 30, INTERVAL_MS);  // N repetitions : Coverage time divided by the number of seconds beetwen each iteration, 30 sec between them
}
//------- FUNCTIONS FOR FTP PROCESS -----------------------------------------------------------------------
int downloadAllFilesFTP(){  // Downloads all files from the FTP to the SD card
    char ftpDir[64];

    sprintf(ftpDir, "/PopUpBuoy_%d", idBuoy);
    SerialPrintDebug("Downloading data from ");
    SerialPrintDebugln(ftpDir);
    SerialPrintDebug("init FTP type A...");
    ftp.InitFile("Type A");
    delay(1000);
    SerialPrintDebugln("done");

    SerialPrintDebug("change FTP directory...");
    ftp.ChangeWorkDir(ftpDir);
    SerialPrintDebugln("done");
    if (!SD.exists(ftpDir)) {
      SerialPrintDebugln("Creating directory " + String(ftpDir) + " in SD");
      SD.mkdir(ftpDir);
    }

    //Change working directory
    SerialPrintDebugln("FTP directory path changed to: " + String(ftpDir));

    String ftpFiles[FTP_BATCH_FILES];
    uint32_t ftpSizes[FTP_BATCH_FILES];

    SerialPrintDebugln("done");

    int offset = 0;
    int nextOffset = 0;
    int nfiles = FTP_BATCH_FILES; // to make sure that we enter in the loop
    int totalFiles = 0;
    int totalBytes = 0;

    int filesDownloaded = 0;
    int filesSkipped = 0;
    int filesFailed = 0;

    int tinit = millis();


    while (nfiles == FTP_BATCH_FILES ) {
      SerialPrintDebug("Getting FTP file list (offset " + String(offset)+ ")...");
      ftp.InitFile("Type A");
      delay(10);

      //===================== GetDirContents torna un espai al principi dels noms!!!! " myfile.txt" ============//
      nfiles = ftp.GetDirContents("", ftpFiles, ftpSizes, FTP_BATCH_FILES, offset, &nextOffset);
      SerialPrintDebug("After  FTP file list offset=" + String(offset) +  " nextOffset=" + String(nextOffset));

      SerialPrintDebugln(" FTP contents: " + String(nfiles) + " files");

      for ( int i = 0 ; i<nfiles; i++ ){
        char dest[512];
        const char* source = ftpFiles[i].c_str();
        sprintf(dest, "%s/%s", ftpDir, source);
        int bytes;
        if ((bytes=tryDownloadFile(source, ftpSizes[i], dest, 3)) < 0){
          SerialPrintDebugln("ERROR in file " + String(source));
          writeLogFile("ERROR downloading file "+ String(source));
          filesFailed += 1;
        }
        else if (bytes == 0) {
          //writeLogFile("File already exists in SD: "+ String(source));
          filesSkipped += 1;
        }
        else {
          totalBytes += bytes;
          //writeLogFile("Downloaded file "+ String(source) + ", size=" + String(ftpSizes[i]));
          filesDownloaded += 1;
        }
      }

      offset = nextOffset;
      totalFiles += nfiles;
    }

    writeLogFile(" ===> Processed " + String(totalFiles) +" files! downloaded=" +  String(filesDownloaded) + " skipped=" + String(filesSkipped) +  " failed=" + String(filesFailed) +  "<====");
    float time = (float)(millis() - tinit)/1000.0;

    if ( time > 0 ) {
        float bitRate = (float)(8*totalBytes/1024)/time; // in Kbytes
        writeLogFile("Total time " + String((millis() - tinit)/1000) + " secs");
        writeLogFile("Bit rate " + String((int)bitRate) + " Kbits/secs");
    }
    if (totalFiles<0){
      return -1;
    }
    return 0;
}
int connectToFTP(){
  #ifdef FTP_SERVER_PRESENT
    SerialPrintDebugln("Connecting to FTP server");
    if (ftp.OpenConnection()<0){      
        writeLogFile("ERROR could not connect to FTP!");
        return -1;
    }
    SerialPrintDebugln("FTP Connection established");
    //check if connection has been established     //TO MODIFY ADD BOOLEAN MEMORY TO KNOW IF IT HAS BEEN CONNECTED ONCE

    if (ftp.isConnected()) {
      digitalWrite(LED_R, HIGH);
    } else {
      digitalWrite(LED_R, LOW);
    }
  #endif
  return 0;
}
int getFileSize(const char* filename) { //Check if a file exists and has exactly the same size
  File file = SD.open(filename);
  if (!file) {
	// File does not exist or could not be opened
	return -1;
  }
  int fileSize = file.size();
  file.close();
  return fileSize;
}
bool fileExistsInSD(const char* filename, uint32_t expectedSize) {

  // Check if the file exists
  if (!SD.exists(filename)) {
	//SerialPrintDebugln("fileExistsInSD -- File [" + String(filename) + "] does not exist");
    return false;
  }

  // Open the file
  File file = SD.open(filename, FILE_READ);
  if (!file) {
	  //SerialPrintDebugln("fileExistsInSD -- Error opening file [" + String(filename) + "]");
    return false;
  }

  // Get the size of the file
  uint32_t sdSize = file.size();

  // Close the file
  file.close();

  // Compare file size with expected size
  if (sdSize == expectedSize) {
	  //SerialPrintDebugln("fileExistsInSD -- file matches size!");
    return true;
  } else {
	 //SerialPrintDebugln("fileExistsInSD -- file does not match size! (ftp_size=" + String(expectedSize) + " sd_size=" + String(sdSize) + ")");
    return false;
  }
}
int tryDownloadFile(const char* source, int size, const char* dest, int tries){ //Try to download file a number of times, if it failed, skip
	int ret=-1;
	while ( (tries--) && (ret < 0) ) {
		ret = DownloadFile(source, size, dest);
		// Check if download failed
		if (ret < 0 ) {
			SerialPrintDebugln("Failed to download file [" + String(source) + "], tries remaining=" + String(tries) + String(" "));
			delay(20);
			ftp.CloseConnection();
			delay(10);
			ftp.OpenConnection();
			delay(10);
			ftp.InitFile("Type A");
			delay(10);
		  char ftpDir[64];
		  sprintf(ftpDir, "/PopUpBuoy_%d", idBuoy);
		  delay(10);
		  ftp.ChangeWorkDir(ftpDir);
		  delay(10);
		}
	}
  
  if (fileBlinkLed) {
    if (ret < 0 ){
      digitalWrite(LED_R, LOW);
      delay(200);
      digitalWrite(LED_R, HIGH);
      delay(200);
      digitalWrite(LED_R, LOW);
      delay(200);
      digitalWrite(LED_R, HIGH);
    }  
    else {
      digitalWrite(LED_G, HIGH);
      delay(200);
      digitalWrite(LED_G, LOW);
      delay(200);
      digitalWrite(LED_G, HIGH);
      delay(200);
      digitalWrite(LED_G, LOW);
    }
  }

	return ret;
}
int DownloadFile(const char* source, int size, const char* dest){ //Downloads a file from the FTP server to the SD card 
	// Checking if destination file exists and has the same size
  // Return > 0 (file size) if downloaded
 // Return = 0 if file skipped (already in sd card)
 // Return = -1 if error
	if (fileExistsInSD(dest, size)) {
		SerialPrintDebugln("File [" + String(source) + "] already exists in SD! skipping");
		return 0;
	}
  int timeout = size/20;
  timeout = max(1000, timeout);
	SerialPrintDebug("Downloading [" + String(source) + "] to SD (FTP size=" + String(size) + ") setting timeout to " + String(timeout) + " msecs ");
	File outputfile = SD.open(dest, FILE_WRITE);
	if (!outputfile) {
		SerialPrintDebugln("ERROR! could not open file" + String(dest));
		return -1;
	}

	if (!ftp.isConnected()){
		SerialPrintDebugln("ERROR! FTP not connected");
		outputfile.close();
		return -1;
	}
	ftp.InitFile("Type I");
  int retcode = 0;

	if ((retcode=ftp.DownloadFileToSD(source, size, &outputfile, timeout)) < 0) {
    // Return codes: 0 success, -1 FTP not connected, -2 ERROR in FTP command, -3 Timeout

		SerialPrintDebug("ERROR in FTP.DownloadFile ");
    if (retcode == -1 ) {
		  SerialPrintDebugln("FTP not connected");
    } else if (retcode == -2) {
		  SerialPrintDebugln("ERROR in FTP command");
    } else if (retcode == -3) {
		  SerialPrintDebugln("FTP timeout");
    } else {
		  SerialPrintDebugln("Unknown code=" + String(retcode));
    }
		outputfile.close();
		return -1;
	}
	outputfile.close();
	int destSize = getFileSize(dest);
	if (destSize != size) {
		SerialPrintDebugln("ERROR! file size expected=" + String(size) + " but got "+ String(destSize) + ", removing file");
		SD.remove(dest);
    return -1;
	}
	SerialPrintDebugln("success!!");
	return size;
}
//------- FUNCTIONS FOR DATA SENDING -----------------------------------------------------------------------
void SendDataMessage() {
  
  ConnectPeripherals(true, GPS_KIM);  // turn on power to all devices
  
  delay(10);
  writeLogFile("Sending : " + String(kineisdataMessage));
  if (KIM.send_data(kineisdataMessage, sizeof(kineisdataMessage) - 1) == OK_KIM) {
    delay(INTERVAL_SEND_MS);
    writeLogFile("Kim MSG_OK");
  } else {
    writeLogFile("Kim MSG_ERR");
  }
  
  ConnectPeripherals(false, GPS_KIM);  // turn on power to all devices

  delay(10);
  goToSleep((INTERVAL_MS-INTERVAL_SEND_MS)/1000);
}
void readSuccessFile() {
  // Open the file or create it if it does not exist
  progressDataFileSD = SD.open(SD_progress_filename, FILE_READ); // if it does not work get back to FILE_READ
  if (!progressDataFileSD) {  // Checking if the file is open
    SerialPrintDebugln(String(SD_progress_filename) + " couldn't be opened");
    RowProgress = 1;  // If it can't open, put the info to 1 in the progress file
    nbrSendingProgress = 0;
  } else {
    if (progressDataFileSD.size() == 0) {  // Checking if the file is empty, if so, put the progress data to 0
      RowProgress = 1;
      nbrSendingProgress = 0;
    } else {
      SerialPrintDebugln(String(SD_progress_filename) + " has been opened");
      String line;
      while (progressDataFileSD.available()) {  // Using this while to go to the last line of the progressFile and so getting the last line
        line = progressDataFileSD.readStringUntil('\n');
      }
      int rowFile;                                        //variable used to stock the first part of the message, the row
      int NbrSendFile;                                    //variable used to stock the second part of the message, the number of sent messages
      splitLineProgressFile(line, rowFile, NbrSendFile);  // Function used to cut the last line of the file to get the index and also the data
      if (NbrSendFile == MaxNbrMsgSendingDataFile) {      // Here, if the number of sending time is equal to the max of sending, it means we are at the end of the sending and we must go to the next line
        rowFile += 1;
        NbrSendFile = 0;
      }
      RowProgress = rowFile;  // The value that we got in the progressFile is put in the global variables
      nbrSendingProgress = NbrSendFile;
      SerialPrintDebugln("Progressfile : line -> " + String(RowProgress) + " and progress -> " + String(nbrSendingProgress));
    }
  }
  progressDataFileSD.close();
}
void splitLineProgressFile(const String &line, int &row, int &nbrsent) {
  int separatorIndex = line.indexOf(':');  // The line will be cut by the ":" caracter. To change the file , only change here the caracter.

  if (separatorIndex != -1) {
    String rowStr = line.substring(0, separatorIndex);  // This part is to get the index
    row = rowStr.toInt();

    String nbrsentStr = line.substring(separatorIndex + 1);  // This part is to get the data
    nbrsent = nbrsentStr.toInt();
  }
}
void splitLineDataFile(const String &line, int &index, char *data) {

  int separatorIndex = line.indexOf(':');  // The line will be cut by the ":" caracter. To change the file , only change here the caracter.

  if (separatorIndex != -1) {
    String indexStr = line.substring(0, separatorIndex);  // This part is to get the Index
    index = indexStr.toInt();

    String dataStr = line.substring(separatorIndex + 1);  // This part is to get the Index
    strcpy(data, dataStr.c_str());
  }
}
void splitLineSuccessFile(const String &line, char *variableName, int &data) {
  int separatorIndex = line.indexOf('=');  // The line will be cut by the ":" caracter. To change the file , only change here the caracter.

  if (separatorIndex != -1) {
    String VariableNameStr = line.substring(0, separatorIndex);  // This part is to get the name of the variable
    strcpy(variableName, VariableNameStr.c_str());

    String dataStr = line.substring(separatorIndex + 1);  // This part is to get the data
    data = dataStr.toInt();
  }
}
char *GetLineDataFile(int Row) {

  // Open the file
  datamsgSD = SD.open(SD_data_filename, FILE_READ);
  if (!datamsgSD) {  // Cheking if the file is open
    SerialPrintDebugln(String(SD_data_filename) + " couldn't be opened");
  } else {
    SerialPrintDebugln(String(SD_data_filename) + " has been opened");

    while (datamsgSD.available()) {
      String line = datamsgSD.readStringUntil('\n');

      int index;       //variable used to stock the first part of the message, the index of the line
      char data[256];  //variable used to stock the second part of the message, the data of the line

      splitLineDataFile(line, index, data);  //Function used to split the line into the two variables before

      if (index == Row) {  //If the line read is the one we want to read in the code
        char *result = new char[strlen(data) + 1];
        strcpy(result, data);
        SerialPrintDebugln("Row to send --> " + String(line));
        datamsgSD.close();
        return result;
      }
    }
    datamsgSD.close();
    return NULL;  // Return NULL if no index = Row
  }
}
void SendFileKim(int time_to_send) {

  // Open the file
  progressDataFileSD = SD.open(SD_progress_filename, FILE_APPEND);  // Opening the "progress" file in "append" so that we can change its value and so its progress
  if (!progressDataFileSD) {                                        // Cheking if the file is open
    SerialPrintDebugln(String(SD_progress_filename) + " couldn't be opened");
  } else {
    SerialPrintDebugln("The SD card has been opened");

    NbrMsgToSend = time_to_send / (INTERVAL_MS / 1000);           // The time to send is divided by the time (in sec) to send one message (Maybe we should adjust the time a bit)
    while (NbrMsgToSend > 0) {                                    // Looping the instructions until the time is over, until there are no messages to send
      int row = RowProgress;
      while (row <= MaxRowDataFile) {  // Loop to get all the row from the data_file
        if (NbrMsgToSend <= 0) {                                  // Condition about the timer to stop sending messages
          RowProgress = row;
          break;
        } else if (row == MaxRowDataFile - 1) {  //If we arrive at the end of the file but with still some time, we start again to read it
          new_line = GetLineDataFile(row);       // We get the line corresponding to the row in the progressFile
          //RowProgress = 0;                       // As we have still some time, we put RowProgress back to 0 to start again at the beginning the new "sending messages" part
        } else {
          new_line = GetLineDataFile(row);  // We get the line corresponding to the row in the progressFile
        }
        for (int N = nbrSendingProgress; N < MaxNbrMsgSendingDataFile; N++) {
          if (NbrMsgToSend <= 0) {  // Condition about the timer to stop sending messages
            nbrSendingProgress = N;
            if (N != 0) {  // If N = 0, it means that we are coming from the last sending of the previous row so the progress file has already been updated
              SerialPrintDebugln("Saving data, end of time ");
              SaveInProgressFile(row, N);  // We save in the progressfile where we are when the time is over
            }
            break;
          } else if (N == MaxNbrMsgSendingDataFile - 1) {  //When we arrive at the end of the sending of a row, put variable to 0 for the next row.
            nbrSendingProgress = 0;
            SerialPrintDebugln(" The line to send is : " + String(new_line));
            strncpy(kineisdataMessage, new_line, sizeof(kineisdataMessage) - 1);
            //kineisdataMessage[sizeof(kineisdataMessage) - 1] = '\0';
            SendDataMessage();
            NbrMsgToSend -= 1;  // One message is sent so we can reduce the counter
            SerialPrintDebugln(" Saving data, end of repetition ");
            SaveInProgressFile(row, N + 1);  // We save in the progressfile where we are when the line is going to change
          } else {
            SerialPrintDebugln(" The line to send is : " + String(new_line));
            strncpy(kineisdataMessage, new_line, sizeof(kineisdataMessage) - 1);
            //kineisdataMessage[sizeof(kineisdataMessage) - 1] = '\0';
            SendDataMessage();
            NbrMsgToSend -= 1;  // One message is sent so we can reduce the counter
          }
        }
        SerialPrintDebugln("End of repetition");
        row++;
      }
      SerialPrintDebugln("End of the file");
      readSuccessFile();
      break;
    }
    delete[] new_line;
    SerialPrintDebugln("End of the sending time. Closing the files");
    progressDataFileSD.close();  // Close the file
  }
}
void SaveInProgressFile(int CurrentRow, int CurrentNbrSent) {
  // Open the file
  progressDataFileSD = SD.open(SD_progress_filename, FILE_APPEND);
  if (!progressDataFileSD) {  // Checking if the file is open
    SerialPrintDebugln(String(SD_progress_filename) + " couldn't be opened");
  } else {
    SerialPrintDebugln(String(SD_progress_filename) + " has been opened");

    if (CurrentRow == MaxRowDataFile && CurrentNbrSent == MaxNbrMsgSendingDataFile) {  // We are at the end of the file, so we must erase the progressFile for the next sendings
      progressDataFileSD.println(String(CurrentRow) + ":" + String(CurrentNbrSent));
      String line = String(CurrentRow) + ":" + String(CurrentNbrSent);
      SerialPrintDebugln("We just add an other line --> " + String(line));
      writeLogFile("We are at the end of the file");
      progressDataFileSD.close();
      //SD.remove(SD_progress_filename); --> act here
    } else {
      progressDataFileSD.println(String(CurrentRow) + ":" + String(CurrentNbrSent));
      String line = String(CurrentRow) + ":" + String(CurrentNbrSent);
      SerialPrintDebugln("We just add an other line --> " + String(line));
      progressDataFileSD.close();
    }
  }
}
void countLinesInDataFile() {
  int lineCount = 0;

  datamsgSD = SD.open(SD_data_filename, FILE_READ);
  if (!datamsgSD) {  // Cheking if the file is open
    SerialPrintDebugln(String(SD_data_filename) + " couldn't be opened and it's lenght is not available");
  } else {
    SerialPrintDebugln(String(SD_data_filename) + " has been opened");
    while (datamsgSD.available()) {
      if (datamsgSD.read() == '\n') {
        lineCount++;
      }
    }
    datamsgSD.close();
  }
  MaxRowDataFile = lineCount;

  SerialPrintDebugln("The numer of MaxRowDataFile is : "+ String(MaxRowDataFile));
}
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
        SerialPrintDebugln("Id Buoy: " + String(DataFromVariable));
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
        SerialPrintDebugln("Time to Sleep safter WiFi attempt: " + String(DataFromVariable) + " minutes");
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

      if (VariableNameStr == "FRM_SLEEP_s") {
        FRMsleepTime_s = DataFromVariable;
        SerialPrintDebugln("Sleep time between GPS transmissions at stage 6: " + String(DataFromVariable) + " seconds");
      }

      if (VariableNameStr == "FRM_SLEEP_NOGPS_s") {
        FRMsleepTime_fail_s = DataFromVariable;
        SerialPrintDebugln("Sleep time at stage 6 when GPS fail: " + String(DataFromVariable) + " seconds");
      }

      if (VariableNameStr == "MinElev") {  // NUEVA VARIABLE FLOAT
        MinElev = static_cast<float>(DataFromVariable);  // Conversión explícita
        SerialPrintDebugln("Minimum Elevation: " + String(MinElev));
      }

      if (VariableNameStr == "PWR2") {  // LEEMOS COMO ENTERO Y LO CONVERTIMOS A CHAR[]
        sprintf(PWR2, "%d", DataFromVariable);
        SerialPrintDebugln("PWR2: " + String(PWR2));
      }

      if (VariableNameStr == "PWR3") {  // LEEMOS COMO ENTERO Y LO CONVERTIMOS A CHAR[]
        sprintf(PWR3, "%d", DataFromVariable);
        SerialPrintDebugln("PWR3: " + String(PWR3));
      }


      if (VariableNameStr == "FILE_BLINK_LED") {
        fileBlinkLed = DataFromVariable;
        fileBlinkLed = false;
        SerialPrintDebugln("Debug FTP file download with LEDs: " + String(DataFromVariable));
      }

      // To add other lines in the file, just follow the same architecture with the "=" in the middle and add here an else if with the right condition
    }
  }
  ConfigFileSD.close();
}
//------- FUNCTIONS FOR AOP TABLE GENERATION-----------------------------------------------------------------------
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
void eraseFolderContent(const char* folderName) {
  File folder = SD.open(folderName);
  
  while (true) {
    File entry =  folder.openNextFile();
    if (!entry) {
      // No more files
      break;
    }
    entry.close();
    SD.remove(entry.name()); // Remove the file    
  }
  
  folder.close();
}


void deleteFilesInFolder(const char* folderPath) {
  File dir = SD.open(folderPath);
  if (!dir || !dir.isDirectory()) {
    Serial.println("Folder not found or not a directory.");
    return;
  }

  File entry;
  while ((entry = dir.openNextFile())) {
    String name = entry.name();
    if (entry.isDirectory()) {
      // Recursively delete contents inside subfolders
      deleteFilesInFolder(name.c_str());
      SD.rmdir(name.c_str());
      Serial.print("Deleted folder: ");
      Serial.println(name);
    } else {
      if (SD.remove(name.c_str())) {
        Serial.print("Deleted file: ");
        Serial.println(name);
      } else {
        Serial.print("Failed to delete: ");
        Serial.println(name);
      }
    }
    entry.close();
  }

  dir.close();
}