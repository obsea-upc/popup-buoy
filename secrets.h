
//#define ESP32_POP_UP

// ---- DATA to CONNECT TO WIFI-------
#ifdef WORK_office
  #define WIFI_SSID "OBSEA_BUOY_WiFi"  //this is the wifi for state 0 and 1 (lab)
  #define WIFI_PASS "sartisarti."
#elif defined WORK_home
  #define WIFI_SSID "Chu_Net"
  #define WIFI_PASS "RyC_84_Wosim"
#elif defined WORK_office2
  #define WIFI_SSID "SARTI_Wifi_v2" //"iPhone de Matias" //
  #define WIFI_PASS "12345612345612345612345612"//"sartisarti."//
#elif defined WORK_iphone
  #define WIFI_SSID "iPhone de Matias" //"intothedeep" //
  #define WIFI_PASS "sartisarti."//"intothedeep"//
#elif defined WORK_intothedeep
  #define WIFI_SSID "intothedeep" //
  #define WIFI_PASS "intothedeep"//

#endif

#ifdef WORK_rasp   
  #define WIFI_SSID2 "pop-up-server-wlan2"  //this is the wifi for state 2 (ocean)
  #define WIFI_PASS2 "plome2023"
#endif

// --- DATA to CONNECT to FTP SERVER-------
#define SECRET_FTP_SERVER_IP "192.168.5.1"//"192.168.0.12"
#define SECRET_FTP_SERVER_USER "pop"  //set in the server  "esp32"
#define SECRET_FTP_SERVER_PASS "plome2023"  //set in the server "esp32"
#define SECRET_FTP_SERVER_PORT 5000     //port on the http

//#define SECRET_FTP_SERVER_IP "192.168.0.36";
//#define SECRET_FTP_SERVER_USER "matias"; //set in the server
//#define SECRET_FTP_SERVER_PASS "matias"; //set in the server
