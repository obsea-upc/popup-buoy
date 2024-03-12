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



#include <WiFiClient.h>
#include "ESP32_FTPClient.h"
#include "SD.h"

ESP32_FTPClient::ESP32_FTPClient(char* _serverAdress, uint16_t _port, char* _userName, char* _passWord, uint16_t _timeout, uint8_t _verbose) {
  userName = _userName;
  passWord = _passWord;
  serverAdress = _serverAdress;
  port = _port;
  timeout = _timeout;
  verbose = _verbose;
}

ESP32_FTPClient::ESP32_FTPClient(char* _serverAdress, char* _userName, char* _passWord, uint16_t _timeout, uint8_t _verbose) {
  userName = _userName;
  passWord = _passWord;
  serverAdress = _serverAdress;
  port = 21;
  timeout = _timeout;
  verbose = _verbose;
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

void ESP32_FTPClient::GetFTPAnswer (char* result, int offsetStart) {
  char thisByte;
  outCount = 0;

  unsigned long _m = millis();
  while (!client.available() && millis() < _m + timeout) delay(1);

  if ( !client.available()) {
    memset( outBuf, 0, sizeof(outBuf) );
    strcpy( outBuf, "Offline");

    _isConnected = false;
    isConnected();
    return;
  }

  while (client.available()) {
    thisByte = client.read();
    if (outCount < sizeof(outBuf)) {
      outBuf[outCount] = thisByte;
      outCount++;
      outBuf[outCount] = 0;
    }
  }
  /*Serial.print("SIZE OF OUTBUF");
    for(int i = offsetStart; i<sizeof(outBuf); i++){
     Serial.println(outBuf[i]);
    }*/


  if (outBuf[0] == '4' || outBuf[0] == '5' ) {
    _isConnected = false;
    isConnected();
    return;
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
    Serial.print("Answer from Server: ");
    Serial.println(result);
    FTPdbg(outBuf);
    FTPdbgn("Result end");
  }
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

void ESP32_FTPClient::OpenConnection() {
  FTPdbg(F("Connecting to: "));
  FTPdbgn(serverAdress);
  if ( client.connect(serverAdress, port, timeout) )
  {
    FTPdbgn(F("Command connected"));
  }
  Serial.print("Welcome message: ");
  delay (1000);
  GetFTPAnswer();
  Serial.println(outBuf);


  FTPdbgn("Send USER");
  client.print(F("USER "));
  client.println(F(userName));
  delay (1000);
  GetFTPAnswer();
  Serial.print("Send User Response: ");
  Serial.println(outBuf);


  FTPdbgn("Send PASSWORD");
  client.print(F("PASS "));
  client.println(F(passWord));
  delay (1000);
  GetFTPAnswer();
  Serial.print("Send PASSWORD Response: ");
  Serial.println(outBuf);


  /*FTPdbgn("Send SYST");
    client.println(F("SYST"));
    delay (500);
    GetFTPAnswer();
    Serial.print("Send SYST Response: ");
    Serial.println(outBuf);
  */
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
    Serial.print("not connected");
    return;
  }
  FTPdbgn(type);
  client.println(F(type));
  delay(1000);
  GetFTPAnswer();
  Serial.print("Send TYPE Response: ");
  Serial.println(outBuf);

  FTPdbgn("Send PASV");
  client.println(F("PASV"));
  delay(1000);
  GetFTPAnswer();
  Serial.print("Send PASV Response: ");
  Serial.println(outBuf);
  char *tStr = strtok(outBuf, "(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL, "(,");
    if (tStr == NULL) {
      FTPdbgn(F("Bad PASV Answer"));
      Serial.println("fucked communication");// Stevie
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
  delay(1000);

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
      //FTPdbgn(String(_b) + ":" + list[_b]);
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

void ESP32_FTPClient::ContentList2(const char * dir, char * client_file_list) {
  char _resp[ sizeof(outBuf) ];
  uint16_t _b = 0;
  String temp_list[128]; //Hi ha un problema amb el pass by reference de list. la funcio nomes passa list[4] quan ha estat definit a list[128] menjant-se info
  String list[128];

  //----------- START send MLSD instruction------------------------------------
  FTPdbgn("Send MLSD");
  if (!isConnected()) return;
  client.print(F("MLSD"));
  client.println(F(dir));
  delay(1000);
  GetFTPAnswer(_resp);

  //------------ END send MLSD instruction ------------------------------------
  // Convert char array to string to manipulate and find response size
  // each server reports it differently, TODO = FEAT
  //String resp_string = _resp;
  //resp_string.substring(resp_string.lastIndexOf('matches')-9);
  //FTPdbgn(resp_string);

  Serial.print("1 SIZE OF list ");
  Serial.println(sizeof(list));
  Serial.print("1 SIZE OF temp list ");
  Serial.println(sizeof(temp_list));

  unsigned long _m = millis();
  while ( !dclient.available() && millis() < _m + timeout) delay(1);

  Serial.print("2 SIZE OF list ");
  Serial.print(sizeof(list));

  while (dclient.available()) {

    if ( _b < 128 ) {
      list[_b] = dclient.readStringUntil('\n');
      temp_list[_b] = list[_b];
      //FTPdbgn(String(_b) + ":" + list[_b]);
      _b++;
      Serial.print("SIZE OF _b:  ");
      Serial.print(sizeof(_b));
      Serial.print(" current val:  ");
      Serial.println(_b);
    }
  }

  Serial.print("SIZE OF temp_list");
  Serial.println(sizeof(temp_list));
  Serial.print("SIZE OF list");
  Serial.println(sizeof(list));

  for (int i = 0; i < _b; i++) {
    Serial.print("temp_list");
    Serial.println(temp_list[i]);
    Serial.println("list");
    Serial.println(list[i]);
  }
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
  Serial.print("Preparing to download:   ");
  Serial.print(filename);
  Serial.print("  size  ");
  Serial.println(length);

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
    Serial.println("Alles good");
    if ( !printUART ) {
      //dclient.readBytes(buf, length); //original code
      Serial.print("inside first");
      dclient.readBytes(buf2, 1024);

      //for(int j = 0; j < 1028; j++ ){
      //Serial.println(j);
      //dclient.readBytes(_buf, 1028);
      //buf2[j]=_buf[0];
      //}
      Serial.println("finished");

    }
    else {
      for (size_t _b = 0; _b < length; _b++ ) {
        Serial.print("inside for");
        Serial.println(_b);
        dclient.readBytes(_buf, 1);
        Serial.print(_buf[0], HEX);
      }
    }
    for (int j = 0; j < 1024; j++ ) {
      Serial.print(buf2[j], HEX);

    }
    Serial.println("DONE");
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

  
  Serial.print("Preparing to download:   ");
  Serial.print(filename);
  Serial.print("  size  ");
  Serial.println(length);


  // create file
  Serial.print("Creating " );
  Serial.print(filename);
  Serial.println("in SD");
  
  File downloaded_file;
  
  downloaded_file = SD.open(filename, FILE_WRITE);  //filename is the file name to be created and FILE_WRITE is a command to create file.
  if(downloaded_file){
  // the file has been opened correctly
    Serial.print(filename);
    Serial.println(" has been opened correctly " ); 
    downloaded_file.close();  //Closing the file
  }else{
    Serial.print(filename);
    Serial.println(" has NOT been opened correctly " );
    downloaded_file.close();  //Closing the file
  }
 

  //check if file was properly created
  if (SD.exists(filename)) {
    Serial.print(filename);
    Serial.println(" exists in SD " );
  } else {
    Serial.print(filename);
    Serial.println(" does NOT exist in SD" );
  }

  Serial.print("Opening " );
  Serial.println(filename);
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
    Serial.print("Client is availiable--  ");
    Serial.println(dclient.available());
    
    Serial.print("receiving from client  --");
    int16_t nb = dclient.readBytes((uint8_t*)inBuffer, FTP_BUF_SIZE); // read from the wifi connection
    if( nb > 0 ){
      // Serial.println( millis() << " " << nb << endl;
      bytes_written=downloaded_file.write((uint8_t*) inBuffer, nb );
      bytesTransfered += nb;
    }
    loop_counter=loop_counter+1;
      
    Serial.print("CLIENT DBG: loop iteration: ");
    Serial.print(loop_counter);          
    Serial.print("--bytes transfered: ");
    Serial.println(bytesTransfered);
    delay(300);
  }
  
  // Closing file
  downloaded_file.close();  //Closing the file
  Serial.print("DONE downloading ");
  Serial.println(filename);
  

}
