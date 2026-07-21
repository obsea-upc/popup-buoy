#include "satellite_tx.h"
#include "conf.h"
#include "logging.h"
#include "power_sleep.h"    // for goToSleep
#include "eeprom_store.h"   // for eepromReadState
#include "KIM.h"
#include <FastCRC.h>
#include <SD.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern KIM KIM;
extern char PWR2[10];
extern char PWR3[10];
extern char AFMT[2];   // "1" -> 2 bytes; sized so sizeof(AFMT) works here
extern const int delayKIM;
extern char kineisMessage[27];
extern char kineisdataMessage[47];
extern char *new_line;
extern int currentState;

extern double gpsLat, gpsLong;
extern uint32_t epochTime;
extern char ADCreadHex[3];

extern int NbrMsgToSend;
extern int RowProgress;
extern int nbrSendingProgress;
extern int MaxRowDataFile;
extern int MaxNbrMsgSendingDataFile;

extern File progressDataFileSD;   // shared with the sketch (createProgressFile)

// Data + progress files owned by this module (filenames declared extern in satellite_tx.h).
char *SD_data_filename;
const char *SD_progress_filename = "/progressFile.txt";
File datamsgSD;

void configureKIM(){
  SerialPrintDebugln("KIM Initial Setup ---->");
  while (!KIM.check()) {
    SerialPrintDebugln("Failed connexion to KIM module. Retriying in 3s...");
    delay(1000);
  }
  if(currentState != ST_FRM){  // all states except Fast Recovery Mode (6) transmit at PWR2
    if (KIM.set_PWR(PWR2, strlen(PWR2)) == OK_KIM) {
      writeLogFile("KIM power changed to: " + String(KIM.get_PWR()));
    } else {
      writeLogFile("Kim Configuration_ERR");
    }
    delay(delayKIM);
  }else{
    if (KIM.set_PWR(PWR3, strlen(PWR3)) == OK_KIM){
      writeLogFile("KIM power changed to: " + String(KIM.get_PWR()));
    } else {
      writeLogFile("Kim Configuration_ERR");
    }
    delay(delayKIM);
  }
  delay(delayKIM);                    // IMPORTANT because by default AT+AFMT=0 and then it sends RAW messages
  if (KIM.set_AFMT(AFMT, sizeof(AFMT) - 1) == OK_KIM) {
    writeLogFile("Kim Configuration changed to AFMT");
  } else {
    writeLogFile("Kim Configuration_ERR");
  }
  delay(delayKIM);
}

bool sendGPSviaKIM(int sendRepeat, int waitRepeat) {

  for (int i = 0; i < sendRepeat; i++) {
    currentState = eepromReadState();
    if (KIM.send_data(kineisMessage, sizeof(kineisMessage) - 1) == OK_KIM) {
      delay(INTERVAL_SEND_MS);
      writeLogFile(" Kim MSG_OK");
    } else {
      writeLogFile(" Kim MSG_ERR");
    }
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

void SendGPSMessage(int timeSending) {

  maskGPS(gpsLat, gpsLong, epochTime, kineisMessage, ADCreadHex);
  sendGPSviaKIM(timeSending / 30, INTERVAL_MS);  // N repetitions : Coverage time divided by the number of seconds beetwen each iteration, 30 sec between them
}

void SendDataMessage() {
  writeLogFile("Sending : " + String(kineisdataMessage));
  if (KIM.send_data(kineisdataMessage, sizeof(kineisdataMessage) - 1) == OK_KIM) {
    delay(INTERVAL_SEND_MS);
    writeLogFile("Kim MSG_OK");
  } else {
    writeLogFile("Kim MSG_ERR");
  }
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
