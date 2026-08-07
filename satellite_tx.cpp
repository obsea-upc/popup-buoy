#include "satellite_tx.h"
#include "conf.h"
#include "logging.h"
#include "power_sleep.h"    // for goToSleep
#include "eeprom_store.h"   // for eepromReadState
#include "sat_module.h"     // KIM1 / Arribada abstraction; nothing here talks to a driver directly
#include <SD.h>

// Objects/data owned by other modules.
extern int currentState;
extern double gpsLat, gpsLong;
extern uint32_t epochTime;
extern char ADCreadHex[3];

// KIM transmission parameters owned by this module (PWR2/PWR3/AFMT/delayKIM declared extern in satellite_tx.h).
char PWR2[10] = "1000";  // TX power (raised from 500 to 1000)
char PWR3[10] = "100";
char AFMT[] = "1";       // enable standard KIM messages
extern const int delayKIM = 10;  // delay between KIM parameter sets
char kineisMessage[27];      // GPS/position message buffer (internal)
char kineisdataMessage[47];  // seabed-data message buffer (internal)
char *new_line;              // current data line being sent (internal)

// Data-transmission progress owned by this module (RowProgress/MaxRowDataFile/MaxNbrMsgSendingDataFile in satellite_tx.h).
int NbrMsgToSend;            // (internal)
int RowProgress;
int nbrSendingProgress;      // (internal)
int MaxRowDataFile;
int MaxNbrMsgSendingDataFile;

extern File progressDataFileSD;   // shared with the sketch (createProgressFile)

// Data + progress files owned by this module (filenames declared extern in satellite_tx.h).
char *SD_data_filename;
const char *SD_progress_filename = "/progressFile.txt";
File datamsgSD;

void configureKIM(){
  SerialPrintDebugln("Satellite module Initial Setup ---->");

  // Works out whether a KIM1 or an Arribada wing is in the socket the first
  // time it runs; afterwards it just returns the cached answer.
  satModuleDetect();

  while (!satModuleCheck()) {
    SerialPrintDebugln("Failed connexion to satellite module. Retriying in 3s...");
    delay(1000);
  }

  // All states except Fast Recovery Mode (6) transmit at PWR2.
  // On the Arribada this is a no-op: its firmware has no AT+PWR.
  const char *power = (currentState != ST_FRM) ? PWR2 : PWR3;
  if (satModuleSetPower(power)) {
    writeLogFile(String(satModuleName()) + " power set to: " + String(power));
  } else {
    writeLogFile("Sat Configuration_ERR (power)");
  }
  delay(delayKIM);

  delay(delayKIM);                    // IMPORTANT because by default AT+AFMT=0 and then it sends RAW messages
  if (satModuleSetFormat(AFMT)) {
    writeLogFile("Sat Configuration changed to AFMT");
  } else {
    writeLogFile("Sat Configuration_ERR (format)");
  }
  delay(delayKIM);
}

// millis() at the end of the previous transmission cycle, i.e. when the last
// sleep ended. Used to keep the spacing between transmissions constant.
static uint32_t cycleStartMillis = 0;

// Sleeps for what is left of a cycleMs-long cycle, counting from when the
// previous one ended, so messages go out every cycleMs however long the GPS
// search and the module dialogue took. Before this, a slow fix and a slow reply
// were simply added on top of a full-length sleep, and FRM could go a minute and
// a half between messages while trying to transmit as often as possible.
static void sleepRestOfCycle(int cycleMs) {
  const uint32_t spent = millis() - cycleStartMillis;
  int32_t sleepMs = (int32_t)cycleMs - (int32_t)spent;
  if (sleepMs < FRM_MIN_SLEEP_MS) sleepMs = FRM_MIN_SLEEP_MS;

  goToSleep(sleepMs / 1000);
  cycleStartMillis = millis();
}

bool sendGPSviaKIM(int sendRepeat, int waitRepeat) {

  for (int i = 0; i < sendRepeat; i++) {
    currentState = eepromReadState();
    if (satModuleSendData(kineisMessage)) {
      delay(INTERVAL_SEND_MS);
      writeLogFile(" " + String(satModuleName()) + " MSG_OK");
    } else {
      writeLogFile(" " + String(satModuleName()) + " MSG_ERR");
    }
    sleepRestOfCycle(waitRepeat);
  }
  return true;
}

void maskGPS(double &gpsLat, double &gpsLong, uint32_t &epochTime, char *kineisMessage, char *ADCreadHex) {
  // this function masks the gps latitute and longitude in hexadecimal
  int latitude, longitude;
  char maskedData[25];
  char hex_latitude[9], hex_longitude[9];
  char hex_epochTime[9];

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

  // Append the battery byte. No CRC is computed here: the module always runs in standard
  // message format (AFMT=1,16,32), where Kineis puts the CRC in the message header itself.
  sprintf(kineisMessage, "%s%s", maskedData, ADCreadHex);

  writeLogFile("Message to transmitt kineisMessage: ");
  writeLogFile(kineisMessage);

  // Reinicia el buffer para la próxima conversión
  memset(hex_longitude, 0, sizeof(hex_longitude));
  memset(hex_latitude, 0, sizeof(hex_latitude));
  memset(hex_epochTime, 0, sizeof(hex_epochTime));
}

void SendGPSMessage(int timeSending) {

  maskGPS(gpsLat, gpsLong, epochTime, kineisMessage, ADCreadHex);

  // FRM and DM space their messages differently (see conf.h), so the number of
  // repetitions has to be worked out from the interval actually in use rather
  // than from a hard-coded 30 s.
  const int cycleMs = (currentState == ST_FRM) ? FRM_INTERVAL_MS : INTERVAL_MS;

  // N repetitions: the coverage time divided by the length of one cycle. FRM
  // calls this with timeSending=30 for a 60 s cycle, which rounds down to zero,
  // so keep at least one - FRM sends exactly one message per loop iteration and
  // does its own repeating.
  int sendRepeat = timeSending / (cycleMs / 1000);
  if (sendRepeat < 1) sendRepeat = 1;

  sendGPSviaKIM(sendRepeat, cycleMs);
}

void SendDataMessage() {
  writeLogFile("Sending : " + String(kineisdataMessage));
  if (satModuleSendData(kineisdataMessage)) {
    delay(INTERVAL_SEND_MS);
    writeLogFile(String(satModuleName()) + " MSG_OK");
  } else {
    writeLogFile(String(satModuleName()) + " MSG_ERR");
  }
  // Seabed data only ever goes out in DM, so it keeps the 30 s DM spacing.
  sleepRestOfCycle(INTERVAL_MS);
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
            kineisdataMessage[sizeof(kineisdataMessage) - 1] = '\0';  // strncpy does not terminate on a full copy, and the send path now measures with strlen()
            SendDataMessage();
            NbrMsgToSend -= 1;  // One message is sent so we can reduce the counter
            SerialPrintDebugln(" Saving data, end of repetition ");
            SaveInProgressFile(row, N + 1);  // We save in the progressfile where we are when the line is going to change
          } else {
            SerialPrintDebugln(" The line to send is : " + String(new_line));
            strncpy(kineisdataMessage, new_line, sizeof(kineisdataMessage) - 1);
            kineisdataMessage[sizeof(kineisdataMessage) - 1] = '\0';  // strncpy does not terminate on a full copy, and the send path now measures with strlen()
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
