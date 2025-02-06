/*
 *  FTP CLIENT FOR ESP32
 * 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//  2023: modified by @Screations
//  2024: modified by UPC


#include "conf.h"
#include <WiFiClient.h>
#include "ESP32_FTPClient.h"
#include "SD.h"


#ifdef FTP_SERIAL_DEBUG
  #define FTPserialDebug Serial.print
  #define FTPserialDebugln Serial.println
#else
  #define FTPserialDebug
  #define FTPserialDebugn
#endif


ESP32_FTPClient::ESP32_FTPClient(char* _serverAdress, uint16_t _port, char* _userName, char* _passWord, uint16_t _timeout, uint8_t _verbose) {
  userName = _userName;
  passWord = _passWord;
  serverAdress = _serverAdress;
  port = _port;
  timeout = _timeout;
  verbose = _verbose;
  outCount = 0;
  bytesTransfered = 0;
  memset(outBuf, 0, 128);
}

ESP32_FTPClient::ESP32_FTPClient(char* _serverAdress, char* _userName, char* _passWord, uint16_t _timeout, uint8_t _verbose) {
  userName = _userName;
  passWord = _passWord;
  serverAdress = _serverAdress;
  port = 21;
  timeout = _timeout;
  verbose = _verbose;
  outCount = 0;
  bytesTransfered = 0;
  memset(outBuf, 0, 128);
}

WiFiClient* ESP32_FTPClient::GetDataClient() {
  return &dclient;
}

bool ESP32_FTPClient::isConnected() {
  if (!_isConnected)
  {
    FTPerr("FTP error: ");
    FTPerr(outBuf);
    FTPerr("\n");
  }

  return _isConnected;
}

void ESP32_FTPClient::GetLastModifiedTime(const char  * fileName, char* result) {
  FTPdbgn("Send MDTM");
  if (!isConnected()) return;
  client.print(F("MDTM "));
  client.println(F(fileName));
  GetFTPAnswer (result, 4);
}

void ESP32_FTPClient::WriteClientBuffered(WiFiClient* cli, unsigned char * data, int dataLength) {
  if (!isConnected()) return;

  size_t clientCount = 0;
  for (int i = 0; i < dataLength; i++) {
    clientBuf[clientCount] = data[i];
    //client.write(data[i])
    clientCount++;
    if (clientCount > bufferSize - 1) {
      cli->write(clientBuf, bufferSize);
      clientCount = 0;
    }
  }
  if (clientCount > 0) {
    cli->write(clientBuf, clientCount);
  }
}

bool ESP32_FTPClient::GetFTPAnswer (char* result, int offsetStart) {
  char thisByte;
  outCount = 0;

  unsigned long _m = millis();
  while (!client.available() && millis() < _m + timeout) delay(1);

  if ( !client.available()) {
    memset( outBuf, 0, sizeof(outBuf) );
    strcpy( outBuf, "Offline");

    _isConnected = false;
    isConnected();
    return true;
  }

  while (client.available()) {
    thisByte = client.read();
    if (outCount < sizeof(outBuf)) {
      outBuf[outCount] = thisByte;
      outCount++;
      outBuf[outCount] = 0;
    }
  }
  /*FTPserialDebug("SIZE OF OUTBUF");
    for(int i = offsetStart; i<sizeof(outBuf); i++){
     FTPserialDebugln(outBuf[i]);
    }*/


  if (outBuf[0] == '4' || outBuf[0] == '5' ) {
    _isConnected = false;
    isConnected();
    return true;
  }
  else {
    _isConnected = true;
  }

  if (result != NULL) {
    FTPdbgn("Result start");
    // Deprecated
    for (int i = offsetStart; i < sizeof(outBuf); i++) {
      result[i] = outBuf[i - offsetStart];
    }
    FTPdbg("Result: ");
    FTPserialDebug("Answer from Server: ");
    FTPserialDebugln(result);
    FTPdbg(outBuf);
    FTPdbgn("Result end");
  }
  return true;
}

void ESP32_FTPClient::WriteData (unsigned char * data, int dataLength) {
  FTPdbgn(F("Writing"));
  if (!isConnected()) return;
  WriteClientBuffered(&dclient, &data[0], dataLength);
}

void ESP32_FTPClient::CloseFile () {
  FTPdbgn(F("Close File"));
  dclient.stop();

  if (!_isConnected) return;

  GetFTPAnswer();
}

void ESP32_FTPClient::Write(const char * str) {
  FTPdbgn(F("Write File"));
  if (!isConnected()) return;

  GetDataClient()->print(str);
}

void ESP32_FTPClient::CloseConnection() {
  client.println(F("QUIT"));
  client.stop();
  FTPdbgn(F("Connection closed"));
}

int ESP32_FTPClient::OpenConnection() {
  FTPdbg(F("Connecting to: "));
  FTPdbgn(serverAdress);
  if ( client.connect(serverAdress, port, timeout) )
  {
    FTPdbgn(F("Command connected"));
  }
  else 
  {
    return -1;
  }
  FTPserialDebug("Welcome message: ");
  delay (100);
  GetFTPAnswer();
  FTPserialDebugln(outBuf);


  FTPdbgn("Send USER");
  client.print(F("USER "));
  client.println(F(userName));
  delay (100);
  GetFTPAnswer();
  FTPserialDebug("Send User Response: ");
  FTPserialDebugln(outBuf);


  FTPdbgn("Send PASSWORD");
  client.print(F("PASS "));
  client.println(F(passWord));
  delay (100);
  GetFTPAnswer();
  FTPserialDebug("Send PASSWORD Response: ");
  FTPserialDebugln(outBuf);


  /*FTPdbgn("Send SYST");
    client.println(F("SYST"));
    delay (500);
    GetFTPAnswer();
    FTPserialDebug("Send SYST Response: ");
    FTPserialDebugln(outBuf);
  */
  return 0;
}

void ESP32_FTPClient::RenameFile(char* from, char* to) {
  FTPdbgn("Send RNFR");
  if (!isConnected()) return;
  client.print(F("RNFR "));
  client.println(F(from));
  GetFTPAnswer();

  FTPdbgn("Send RNTO");
  client.print(F("RNTO "));
  client.println(F(to));
  GetFTPAnswer();
}

void ESP32_FTPClient::NewFile (const char* fileName) {
  FTPdbgn("Send STOR");
  if (!isConnected()) return;
  client.print(F("STOR "));
  client.println(F(fileName));
  GetFTPAnswer();
}

void ESP32_FTPClient::SetPassive() { //SC-2022
  FTPdbgn("Send PASV");
  client.println(F("PASV"));
  GetFTPAnswer();

}

void ESP32_FTPClient::SendType(const char* type) { //SC-2022
  FTPdbgn("Send TYPE");
  if (!isConnected()) return;
  FTPdbgn(type);
  client.println(F(type));
  GetFTPAnswer();

}

void ESP32_FTPClient::InitFile(const char* type) {
  FTPdbgn("Send TYPE");
  if (!isConnected()) {
    FTPserialDebug("not connected");
    return;
  }
  FTPdbgn(type);
  client.println(F(type));
  delay(100);
  GetFTPAnswer();
  FTPserialDebug("Send TYPE Response: ");
  FTPserialDebugln(outBuf);

  FTPdbgn("Send PASV");
  client.println(F("PASV"));
  delay(100);
  GetFTPAnswer();
  FTPserialDebug("Send PASV Response: ");
  FTPserialDebugln(outBuf);
  char *tStr = strtok(outBuf, "(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL, "(,");
    if (tStr == NULL) {
      FTPdbgn(F("Bad PASV Answer"));
      FTPserialDebugln("fucked communication");// Stevie
      CloseConnection();
      return;
    }
    array_pasv[i] = atoi(tStr);
  }
  unsigned int hiPort, loPort;
  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;

  IPAddress pasvServer(array_pasv[0], array_pasv[1], array_pasv[2], array_pasv[3]);

  FTPdbg(F("Data port: "));
  hiPort = hiPort | loPort;
  FTPdbgn(hiPort);
  if (dclient.connect(pasvServer, hiPort, timeout)) {
    FTPdbgn(F("Data connection established"));
  }
}

void ESP32_FTPClient::AppendFile (char* fileName) {
  FTPdbgn("Send APPE");
  if (!isConnected()) return;
  client.print(F("APPE "));
  client.println(F(fileName));
  GetFTPAnswer();
}

void ESP32_FTPClient::ChangeWorkDir(const char * dir) {

  FTPdbgn("Send CWD");
  if (!isConnected()) return;
  client.print(F("CWD "));
  client.println(F(dir));
  GetFTPAnswer();
}

void ESP32_FTPClient::DeleteFile(const char * file) {
  FTPdbgn("Send DELE");
  if (!isConnected()) return;
  client.print(F("DELE "));
  client.println(F(file));
  GetFTPAnswer();
}

void ESP32_FTPClient::MakeDir(const char * dir) {
  FTPdbgn("Send MKD");
  if (!isConnected()) return;
  client.print(F("MKD "));
  client.println(F(dir));
  GetFTPAnswer();
}



void ESP32_FTPClient::ContentList(const char * dir, String * list) {
  char _resp[ sizeof(outBuf) ];
  uint16_t _b = 0;
  //String temp_list[128]; //Hi ha un problema amb el pass by reference de list. la funcio nomes passa list[4] quan ha estat definit a list[128] menjant-se info

  FTPdbgn("Send MLSD");
  if (!isConnected()) return;
  client.print(F("MLSD"));
  client.println(F(dir));
  delay(100);

  GetFTPAnswer(_resp);

  // Convert char array to string to manipulate and find response size
  // each server reports it differently, TODO = FEAT
  //String resp_string = _resp;
  //resp_string.substring(resp_string.lastIndexOf('matches')-9);
  //FTPdbgn(resp_string);


  unsigned long _m = millis();
  while ( !dclient.available() && millis() < _m + timeout) delay(1);

  while (dclient.available()) {

    if ( _b < 128 ) {
      list[_b] = dclient.readStringUntil('\n');
      //temp_list[_b]= list[_b];
      FTPserialDebugln("File " + String(_b) + ":" + list[_b]);
      _b++;
      //Serial.print("SIZE OF _b:  ");
      //Serial.print(sizeof(_b));
      //Serial.print(" current val:  ");
      //Serial.println(_b);
    }
  }


  //for(int i = 0; i<_b; i++){
  //   Serial.print("temp_list:    ");
  //   Serial.println(temp_list[i]);
  //   Serial.println("list:    ");
  //   Serial.println(list[i]);
  //}
}

//// type=file;size=7386;modify=20231115215840;UNIX.mode=0644;UNIX.uid=1000;UNIX.gid=1001;unique=b302g21647; crab2.jpg

void parseFileInfo(String input, String* type, String* filename, uint32_t* size) {
    char *token;
    char *size_str = NULL;
    char* file_str = NULL;
    char* type_str = NULL;
    // Copy the input string because strtok modifies the string
    char inputCopy[512];
    strcpy(inputCopy, input.c_str());


    token = strtok(inputCopy, ";");
    while (token != NULL) {
        // Check for the 'type', 'size', and 'filename' fields
        if (strstr(token, "type=") != NULL) {
            type_str = strchr(token, '=') + 1;
        }
        else if (strstr(token, "size=") != NULL) {  // File size
            size_str = strchr(token, '=') + 1;
        }
        else if (strstr(token, "sizd=") != NULL) {  // directory size
            size_str = strchr(token, '=') + 1;
        }
        else if (strchr(token, ' ') != NULL) {
            // 'filename' field (contains space)
        	token++;
            file_str = token;
            break;
        }
        token = strtok(NULL, ";");
    }

    // Erase carriage return at the end
	if (file_str[strlen(file_str) -1] == '\r') {
	  file_str[strlen(file_str) -1] = 0;
	}
	// Convert size string to integer
	*size = atoi(size_str);
	*filename = String(file_str);
	*type = String(type_str);
}


/*
 * Get the list of files and sizes in a directory. To save memory only the first 'buffsize' entries will
 * be processed. To process the rest of the files, call this method again with offset > 0
 *
 * Params:
 *    dir: directory to list
 *    filenames: buffer to store all filenames
 *    sizes: bufer to store all sizes
 *    buffsize: size of filenames and sizes arrays
 *    offset: ignore first x entries
 */
int ESP32_FTPClient::GetDirContents(const char* dir, String* filenames, uint32_t* sizes, int buffsize, int offset, int* nextOffset){
  char _resp[ sizeof(outBuf) ];
  uint16_t i = 0;

  FTPdbgn("Send MLSD");
  if (!isConnected()) return -1;
  client.print(F("MLSD"));
  client.println(F(dir));
  delay(300);
  GetFTPAnswer(_resp);


  // Timeout
  unsigned long _m = millis();
  while ( !dclient.available() && millis() < _m + timeout) delay(1);

  int fileCount = 0;  // processed files
  int skippedFiles = 0;  // files skipped


  FTPserialDebugln("Reading FTP contents...");
  while (dclient.available()) {
    String buff;
    buff = dclient.readStringUntil('\n');
    FTPserialDebug("Iteration " + String(i) + "...");
    if ( fileCount < buffsize ) {
      if (skippedFiles < offset) {
    	  // Skip first n lines
    	    FTPserialDebug("Skip by offset...");
    	    skippedFiles++;

      } else {
          String type;
		  parseFileInfo(buff, &type, &filenames[fileCount], &sizes[fileCount]);

		  if ( type.equals("cdir") || type.equals("pdir") ) {
			  skippedFiles++;
			  FTPserialDebug("Skip " + type);
		  }
		  else {
			 FTPserialDebug("PROCESSING FILE " + filenames[fileCount]);
			 fileCount++;
		  }
      }
    }
    else {
    	FTPserialDebug("skip by buffsize ");
    }
    i++;
    FTPserialDebugln("");
  }
  *nextOffset = skippedFiles + fileCount;
  return fileCount;
}




void ESP32_FTPClient::ContentListWithListCommand(const char * dir, String * list) {
  char _resp[ sizeof(outBuf) ];
  uint16_t _b = 0;


  FTPdbgn("Send LIST");
  if (!isConnected()) return;
  client.print(F("LIST"));
  client.println(F(dir));
  GetFTPAnswer(_resp);

  // Convert char array to string to manipulate and find response size
  // each server reports it differently, TODO = FEAT
  //String resp_string = _resp;
  //resp_string.substring(resp_string.lastIndexOf('matches')-9);
  //FTPdbgn(resp_string);

  unsigned long _m = millis();
  while ( !dclient.available() && millis() < _m + timeout) delay(1);

  while (dclient.available())
  {
    if ( _b < 128 )
    {
      String tmp = dclient.readStringUntil('\n');
      list[_b] = tmp.substring(tmp.lastIndexOf(" ") + 1, tmp.length());
      //FTPdbgn(String(_b) + ":" + tmp);
      _b++;
    }
  }

}

void ESP32_FTPClient::DownloadString(const char * filename, String &str) {
  FTPdbgn("Send RETR");
  if (!isConnected()) return;
  client.print(F("RETR "));
  client.println(F(filename));

  char _resp[ sizeof(outBuf) ];

  GetFTPAnswer(_resp);

  unsigned long _m = millis();
  while ( !GetDataClient()->available() && millis() < _m + timeout) delay(1);

  while ( GetDataClient()->available() )
  {
    str += GetDataClient()->readString();
  }

}

void ESP32_FTPClient::DownloadFile(const char * filename, unsigned char * buf, size_t length, bool printUART ) {
  FTPserialDebug("Preparing to download:   '");
  FTPserialDebug(filename);
  FTPserialDebug("  size  ");
  FTPserialDebugln(length);

  FTPdbgn("Send RETR");
  if (!isConnected()) return;
  client.print(F("RETR "));
  client.println(F(filename));

  char _resp[ sizeof(outBuf) ];
  GetFTPAnswer(_resp);

  char _buf[2];
  char buf2[1024];

  //wait until client is available
  unsigned long _m = millis();
  while ( !dclient.available() && millis() < _m + timeout){
    delay(1);
  }

  while (dclient.available()) {
    FTPserialDebugln("Alles good");
    if ( !printUART ) {
      //dclient.readBytes(buf, length); //original code
      FTPserialDebug("inside first");
      dclient.readBytes(buf2, 1024);

      //for(int j = 0; j < 1028; j++ ){
      //FTPserialDebugln(j);
      //dclient.readBytes(_buf, 1028);
      //buf2[j]=_buf[0];
      //}
      FTPserialDebugln("finished");

    }
    else {
      for (size_t _b = 0; _b < length; _b++ ) {
        FTPserialDebug("inside for");
        FTPserialDebugln(_b);
        dclient.readBytes(_buf, 1);
        FTPserialDebug(_buf[0], HEX);
      }
    }
    for (int j = 0; j < 1024; j++ ) {
      FTPserialDebug(buf2[j], HEX);

    }
    FTPserialDebugln("DONE");
  }
}

void ESP32_FTPClient::DownloadFileAndSaveToSD(const char * filename, unsigned char * buf, size_t length ) {

 /* DownloadFileAndSaveToSD
  *   This function is intended to be used to download large files and directly save them in th SD
  *   It downloads packets of 1024 bytes and sequantialy stores them in the file
  *   arguments:
  *   - filename: name of file to be downloaded
  *   - buf : buffer size
  *   - length: total size of file
  *
  */

  //test files to download
  //filename="/test1.png";


  FTPserialDebug("Preparing to download:   ");
  FTPserialDebug(filename);
  FTPserialDebug("  size  ");
  FTPserialDebugln(length);


  // create file
  FTPserialDebug("Creating " );
  FTPserialDebug(filename);
  FTPserialDebugln("in SD");

  File downloaded_file;

  downloaded_file = SD.open(filename, FILE_WRITE);  //filename is the file name to be created and FILE_WRITE is a command to create file.
  if(downloaded_file){
  // the file has been opened correctly
    FTPserialDebug(filename);
    FTPserialDebugln(" has been opened correctly " );
    downloaded_file.close();  //Closing the file
  }else{
    FTPserialDebug(filename);
    FTPserialDebugln(" has NOT been opened correctly " );
    downloaded_file.close();  //Closing the file
  }


  //check if file was properly created
  if (SD.exists(filename)) {
    FTPserialDebug(filename);
    FTPserialDebugln(" exists in SD " );
  } else {
    FTPserialDebug(filename);
    FTPserialDebugln(" does NOT exist in SD" );
  }

  FTPserialDebug("Opening " );
  FTPserialDebugln(filename);
  // write in files
  downloaded_file = SD.open(filename, FILE_WRITE);


  //Requesting server to send the file
  FTPdbgn("Send RETR");
  if (!isConnected()) return;
  client.print(F("RETR "));
  client.println(F(filename));

  char _resp[ sizeof(outBuf) ];
  GetFTPAnswer(_resp);

  int bytes_written=0;


  //wait until client is available
  unsigned long _m = millis();
  while ( !dclient.available() && millis() < _m + timeout){
    delay(1);

  }

  int loop_counter=0;

  while (dclient.available()) {
    FTPserialDebug("Client is availiable--  ");
    FTPserialDebugln(dclient.available());

    FTPserialDebug("receiving from client  --");
    int16_t nb = dclient.readBytes((uint8_t*)inBuffer, FTP_BUF_SIZE); // read from the wifi connection
    if( nb > 0 ){
      // FTPserialDebugln( millis() << " " << nb << endl;
      bytes_written=downloaded_file.write((uint8_t*) inBuffer, nb );
      bytesTransfered += nb;
    }
    loop_counter=loop_counter+1;

    FTPserialDebug("CLIENT DBG: loop iteration: ");
    FTPserialDebug(loop_counter);
    FTPserialDebug("--bytes transfered: ");
    FTPserialDebugln(bytesTransfered);
    delay(300);
  }

  // Closing file
  downloaded_file.close();  //Closing the file
  FTPserialDebug("DONE downloading ");
  FTPserialDebugln(filename);


}


/*
 *  Return codes: 0 success, -1 FTP not connected, -2 ERROR in FTP command, -3 Timeout
 */
int ESP32_FTPClient::DownloadFileToSD(const char * filename, int length, File* destination, int file_timeout) {
  FTPserialDebugln("Preparing to download:   [" + String(filename) + "] (size " + String(length) + " bytes)");

  //Requesting server to send the file
  if (!isConnected()) {
	  FTPserialDebug("ERROR! FTP not connected");
	  return -1;
  }

  char cmd[256];
  sprintf(cmd, "RETR %s\0", filename);
  client.println(cmd);
  char _resp[ sizeof(outBuf) ];

  if (!GetFTPAnswer(_resp)) {
	  FTPserialDebug("ERROR in FTP command");
	  return -2;
  }

  int bytes_written=0;  
  //wait until client is available
  unsigned long init = millis();

  int loop_counter = 0;
  bool timeout_flag = false;
  while ( bytes_written < length ) {
    while (!dclient.available()) {
      // If client is not available, wait until it is
      delay(1);
      if (millis() > init + file_timeout) {
        // If we waited too long (timeout expired) break the while
        FTPserialDebug("ERROR  FTP TIMEOUT");
        return -3;
      }
    }
	
    int16_t nb = dclient.readBytes((uint8_t*)inBuffer, FTP_BUF_SIZE); // read from the wifi connection
    FTPserialDebug(" ---> " + String(nb) + " bytes read");

    if( nb > 0 ){
      bytes_written += destination->write((uint8_t*) inBuffer, nb );
      bytesTransfered += nb;
    } 
    FTPserialDebug("FTP READ " + String(nb) + " bytes, requested " + String(FTP_BUF_SIZE) + " total written " + String(bytes_written) + " of " + String(length));
    loop_counter=loop_counter+1;    
    FTPserialDebugln("total written " + String(bytes_written)); 
  }
  return 0;
}











