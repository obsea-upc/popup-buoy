#include "ftp_download.h"
#include "conf.h"
#include "logging.h"
#include <WiFi.h>   // ESP32_FTPClient.h uses WiFiClient but doesn't include it
#include "ESP32_FTPClient.h"
#include <SD.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern ESP32_FTPClient ftp;
extern int idBuoy;
extern int fileBlinkLed;

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

  if ( fileBlinkLed == 1) {
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
