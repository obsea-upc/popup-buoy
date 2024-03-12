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
//  2023: modified by @SC.bcn

/*******************************************************************************
 **                                                                            **
 **                       DEFINITIONS FOR FTP CLIENT                           **
 **                                                                            **
 *******************************************************************************/

// Uncomment to print debugging info to console attached to ESP32
#define FTP_DEBUG
//#ifdef FTP_DEBUG
//#endif

#define FTP_CLIENT_VERSION "FTP-2023-02-07"

#define FTP_TIME_OUT  5       // Disconnect client after 5 minutes of inactivity
#define FTP_CMD_SIZE 255 + 8  // max size of a command
#define FTP_CWD_SIZE 255 + 8  // max size of a directory name
#define FTP_FIL_SIZE 255      // max size of a file name
#define FTP_BUF_SIZE 4096 //512   //  700 KByte/s download in AP mode, direct connection.

class ESP32_FTPClient
{
  private:
  void WriteClientBuffered(WiFiClient* cli, unsigned char * data, int dataLength);
  char outBuf[128];
  unsigned char outCount;
  WiFiClient client;
  WiFiClient dclient;
  uint8_t verbose;

  template<typename T>
  		void FTPdbg(T msg) {
    	if(verbose == 2) Serial.print(msg);
		}
  
  template<typename T>
  		void FTPdbgn(T msg) {
    	if(verbose == 2) Serial.println(msg);
		}

    template<typename T>
    void FTPerr(T msg) {
    if(verbose == 1 || verbose == 2) Serial.print(msg);
  }

  char* userName;
  char* passWord;
  char* serverAdress;
  uint16_t port;
  bool _isConnected = false;
  unsigned char clientBuf[1500];
  size_t bufferSize = 1500;
  uint16_t timeout = 10000;
  WiFiClient* GetDataClient();

  //added by SC.bcn
  char inBuffer[ FTP_BUF_SIZE ]; // data buffer for transfers
  uint32_t bytesTransfered;  // bytes transfered
  public:
  ESP32_FTPClient(char* _serverAdress, uint16_t _port, char* _userName, char* _passWord, uint16_t _timeout = 10000, uint8_t _verbose = 1);
  ESP32_FTPClient(char* _serverAdress, char* _userName, char* _passWord, uint16_t _timeout = 10000, uint8_t _verbose = 1);
  void OpenConnection();
  void CloseConnection();
  bool isConnected();
  void NewFile (const char* fileName);
  void AppendFile( char* fileName);
  void WriteData (unsigned char * data, int dataLength);
  void CloseFile ();
  void GetFTPAnswer (char* result = NULL, int offsetStart = 0);
  void GetLastModifiedTime(const char* fileName, char* result);
  void RenameFile(char* from, char* to);
  void Write(const char * str);
  void SetPassive();
  void SendType(const char* type);
  void InitFile(const char* type);
  void ChangeWorkDir(const char * dir);
  void DeleteFile(const char * file);
  void MakeDir(const char * dir);
  void ContentList2(const char * dir, char * client_file_list);
  void ContentList(const char * dir, String * list);
  void ContentListWithListCommand(const char * dir, String * list);
  void DownloadString(const char * filename, String &str);
  void DownloadFile(const char * filename, unsigned char * buf, size_t length, bool printUART = false);
  void DownloadFileAndSaveToSD(const char * filename, unsigned char * buf, size_t length);
};
