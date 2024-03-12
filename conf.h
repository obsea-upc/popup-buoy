////----- TEST PARAMETERS

#define INITIAL_STATE 0

///------ OPTIONS TO DEBUG
#define SERIAL_DEBUG  // comment line to disable Serial prints
#define SERIAL_DEBUG_BAUDRATE 115200

//------- EEPROM
#define EEPROM_SIZE 5

//------ DEFINITIONS for WiFi
//#define WORK_office // comment when working from home
#define WORK_home
//#define WORK_enoc
//#define WORK_office2 
#define WORK_rasp
#define WIFI_TIMEOUT 120000 // 2 minutos en milisegundos

// ----- Definitions for SD card options
//#define SD_CS 5
//#define SDSPEED 40000000

// ----- Definitions for push button
#define PB_1 25  //12
#define PB_2 26  //14
#define PB_3 27  //27

// ----- Definitions for leds
#define LED_R 32  //25
#define LED_Y 33  //33
#define LED_G 0   //32

//------ Definition for power relay
#define GPS_KIM 13  
#define SD_card 14
#define DISCONNECT_PHER  //Disconnect KIM and GPS between transmissions

//------ Definition ADC read
#define ADC_PIN 36  
#define WORK_ADC
#define BAT_CRIT_LEVEL 3.5

//------ Definition for Sleep mode and parameters
#define TIME_LESS_BEFORE_AWAKENING 30  //time (in sec) took from the general time to wait the awakening to be sure not to miss the satellite 
#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 50  
#define TIME_TO_SLEEP_STATE1h 0       //Time to Sleep state 1 (from config to deep sea) in hours
#define TIME_TO_SLEEP_STATE1m 1       //Time to Sleep state 1 (from config to deep sea) in min
#define TIME_TO_SLEEP_STATE3h 0       //Time to Sleep state 3 (from release to surface) in hours
#define TIME_TO_SLEEP_STATE3m 1       //Time to Sleep state 3 (from release to surface) in min
#define TIME_TO_SLEEP_STATE3errorm 1   //Time to Sleep state 3 (if WiFi or HTTP error) in min
#define TIME_TO_SLEEP_ERROR_GPS 300    //Time to sleep when the GPS can't fix- 1st time (s)
#define TIME_TO_SLEEP_ERROR_GPS_RECURRENT 3600  //Time to sleep when the GPS can't fix for multiple times (s)
#define CRIT_FACTOR 3                 // In stage 5, critical battery, all sleep times are X times larger

//RTC_DATA_ATTR unsigned int bootCount = 0;
#define BUTTON_PIN_BITMASK 0x001A  // GPIOs 2 and 15

//------ Definition for External RTC
// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 34

//------ Definition for FTP
#define FTP_SERVER_PRESENT

//------ Definition for GPS module
#define RXPin_GPS 4
#define TXPin_GPS 2  
#define GPSBaud 9600 
#define MAX_GPS_TIMEOUT 200000     // Time used to let the GPS searching to get fixed (ms)

//------ Definition for KINEIS module
#define MAX_SLEEP_TIME_S 1800  //Maximum surface sleep time in s at any condition (to ensure the recovery)
#define TRANSMISSION_GPS_S 150    //Time for normal GPS transmission, minimum --> Minimum duration (default 5 minutes, now at 2.5 min) --> 300 s =10 messages
#define TRANSMISSION_GPS_NOARG_S 90   //Time for GPS transmission, no ARGOS coverage (default 90 -> 3 messages)
#define INTERVAL_MS 30000         // Time in ms between two kineis messages (ms)
#define INTERVAL_SEND_MS 6000    //Boosting the message
#define KIM_RXD0 16
#define KIM_TXD0 17
#define KIMBaud 9600  //4800 in prev KIM1
#define maxAOPSize 10  //Maxim number of satellites in AOP tamble
#define stdMinElev 15.0f
#define critMinElev 30.0f
#define FORCE_A2_UPLINK_STATUS

