#include "config.h"
#include "logging.h"
#include <SD.h>

// SD config file (owned by this module; nothing else touches it).
static const char *SD_config_filename = "/conf.txt";  // key=value config, so parameters can change without recompiling
static File ConfigFileSD;

// Runtime globals owned by the main sketch (popup-buoy.ino), filled from /conf.txt.
extern int MaxNbrMsgSendingDataFile;
extern int idBuoy;
extern int maxGPSTimeout;
extern int maxWIFITimeout;
extern int sleeptime_s1_h;
extern int sleeptime_s1_m;
extern int sleepTimeWifiAttempt;
extern int sleeptime_errorGPS_s;
extern int sleeptime_errorGPS_recurrent_s;
extern int max_sleep_time_s;
extern int timetransm_GPS_s;
extern int timetransm_GPS_noArg_s;
extern unsigned long maxFRM;
extern float MinElev;
extern char PWR2[10];
extern char PWR3[10];
extern int fileBlinkLed;
extern float Bat_critlevel;

// Split a "Name=value" config line into its name and integer value.
static void splitLineSuccessFile(const String &line, char *variableName, int &data) {
  int separatorIndex = line.indexOf('=');  // The line will be cut by the "=" caracter. To change the file, only change here the caracter.

  if (separatorIndex != -1) {
    String VariableNameStr = line.substring(0, separatorIndex);  // This part is to get the name of the variable
    strcpy(variableName, VariableNameStr.c_str());

    String dataStr = line.substring(separatorIndex + 1);  // This part is to get the data
    data = dataStr.toInt();
  }
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
