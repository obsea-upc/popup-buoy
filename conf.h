#pragma once

#define SOFT_VERSION "v0.1.1"
#define COMPILE_DATE __DATE__


//------ Pop-up buoy state machine ------
// Stored as an int in EEPROM (addr 0), so the numbering must stay stable
// across firmware versions (ST_RELEASE=3 is kept so 4/5/6 keep their values).
enum PopUpState {
  ST_CONFIG  = 0,  // first-boot configuration (RTC/SD/FTP/WiFi)
  ST_DEPLOY  = 1,  // wait for PB, then sleep for lander deployment
  ST_SEABED  = 2,  // seabed routine: permission, FTP download, release
  ST_RELEASE = 3,  // unused: release is handled inside ST_SEABED
  ST_DM      = 4,  // surface: Drifting Mode
  ST_LOWPWR  = 5,  // surface: survival / critical battery
  ST_FRM     = 6   // surface: Fast Recovery Mode
};

// Human-readable state name for logs/serial (inline so it can live in this header).
inline const char* stateName(int s) {
  switch (s) {
    case ST_CONFIG:  return "CONFIG";
    case ST_DEPLOY:  return "DEPLOY";
    case ST_SEABED:  return "SEABED";
    case ST_RELEASE: return "RELEASE";
    case ST_DM:      return "DM";
    case ST_LOWPWR:  return "LOWPWR";
    case ST_FRM:     return "FRM";
    default:         return "?";
  }
}

////----- TEST PARAMETERS

#define INITIAL_STATE ST_CONFIG
#define TEST_FORCE_GPS_VILANOVA_PB2  // Test aid (permanent): tap PB_2 during the GPS search to inject a Vilanova i la Geltru fix from the RTC, so the Surface/SPP path can be exercised indoors

////----- OPTIONAL FEATURES

//#define FRM_USV_UPLOAD  // In Fast Recovery Mode, look for the BlueBoat's WiFi on every cycle and,
                          // if the USV is in range, hand it the SD data over FTP and stay in FRM
                          // instead of dropping to DM. Currently OFF: in FRM the buoy should only
                          // acquire a GPS fix and transmit it. Uncomment to bring the USV rendezvous
                          // back; the code it guards lives in the ST_FRM case and in usv_upload.*

///------ OPTIONS TO DEBUG
//#define GPS_DEBUG_NMEA_GSA  // GPS diagnostics, kept for future use: echoes raw NMEA (GSA/PCAS/TXT),
                              // sends a $PCAS06 chipset probe and a UBX test that tries to silence GSA.
                              // The current receiver ignores every command (see gps.cpp); uncomment to
                              // investigate again if a genuine u-blox module is ever fitted.
#define SERIAL_DEBUG  // comment line to disable Serial prints
#define SERIAL_DEBUG_BAUDRATE 115200

//------- EEPROM
#define EEPROM_SIZE 7

//------ DEFINITIONS for WiFi
//#define WORK_office // comment when working from home
//#define WORK_home
//#define WORK_enoc
#define WORK_office2 
#define WORK_rasp
//#define WORK_vela
//#define WORK_intothedeep

// ----- Definitions for push button
#define PB_1 25  //12
#define PB_2 26  //14
#define PB_3 27  //27
//#define PB_3 39 

// ----- Definitions for leds
#define LED_R 32  //25
#define LED_Y 33  //33
#define LED_G 0   //32

//------ Definition for power relay
#define GPS_KIM 13  //future just KIM
#define SD_card 14
//#define GPS 27 future just GPS 
#define DISCONNECT_PHER  //Disconnect KIM and GPS between transmissions

//------ Definition ADC read
#define ADC_PIN 36
//#define BAT_CRIT_LEVEL 3.5

//------ Definition for Sleep mode and parameters
#define TIME_LESS_BEFORE_AWAKENING 30  //time (in sec) took from the general time to wait the awakening to be sure not to miss the satellite
// How many times the pass prediction may fail before the buoy stops retrying and
// deep-sleeps instead. The retry was unbounded and cost both buoys more than five
// hours of the 9 Aug test, awake, recomputing a prediction that could not succeed.
#define SPP_MAX_RETRIES 3
#define SPP_GIVEUP_SLEEP_S 600  // sleep after giving up, then retry with a fresh GPS fix
// How long the peripherals need after their rail comes back before they will
// answer. Measured on the Arribada 9 Aug 2026: it first replied to AT+ID 482 ms
// after power was restored. The 5 ms that used to be here meant the firmware was
// talking to a module that was still booting, so AT+KMAC came back with the boot
// banner instead of +OK and the AT+TX after it was refused with +ERROR=253.
#define SAT_MODULE_BOOT_MS 1000 
#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds 
#define CRIT_FACTOR 3                 // In stage 5, critical battery, all sleep times are X times larger

//RTC_DATA_ATTR unsigned int bootCount = 0;
#define BUTTON_PIN_BITMASK 0x001A  // GPIOs 2 and 15

//------ Definition for External RTC
#define CLOCK_INTERRUPT_PIN 34      // the pin that is connected to SQW

//------ Definition for FTP
#define FTP_SERVER_PRESENT
#define FTP_BATCH_FILES 64 // quantitat de fitxers que et descarregues cada vegada
//#define FTP_SERIAL_DEBUG

//------ Definition for GPS module
#define RXPin_GPS 4
#define TXPin_GPS 2
#define GPSBaud 9600

//------ Definition for KINEIS module
// Spacing between two satellite transmissions, measured transmission to
// transmission. The sleep absorbs however long the GPS search and the module
// dialogue took, so the spacing holds whatever the cycle costs - see
// sleepRestOfCycle() in satellite_tx.cpp.
//
#define INTERVAL_MS 30000        // DM + seabed data: ms between transmissions
#define FRM_INTERVAL_MS 30000    // Fast Recovery Mode: ms between transmissions
#define INTERVAL_SEND_MS 6000    //Boosting the message
// Floor for the sleep between transmissions. The sleep is whatever is left of
// the cycle after the GPS search and the module dialogue, so on a slow cycle it
// shrinks towards this value instead of being added on top of a full-length one.
#define FRM_MIN_SLEEP_MS 5000
#define KIM_RXD0 16
#define KIM_TXD0 17
#define KIMBaud 9600  //4800 in prev KIM1
#define maxAOPSize 30  //Maxim number of satellites in AOP tamble
//#define stdMinElev 20.0f                                                                                                                                 // SD
#define critMinElev 45.0f                                                                                                                               // SD
#define FORCE_A2_UPLINK_STATUS
