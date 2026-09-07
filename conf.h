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
// Shortest remaining slice of a pass still worth restarting the transmit cycle
// for. Matias's call, 10 Aug 2026: keep this at 70 s. Dropping it to one message
// was tried and rejected - losing the last thirty seconds of a pass costs almost
// nothing, and waking the whole cycle again to send a single message is not worth
// the power. The ten SPP errors of the 9-10 Aug run all fall below this and are
// accepted as normal.
#define SPP_MIN_USABLE_COVERAGE_S 70
#define SPP_MAX_RETRIES 3
#define SPP_GIVEUP_SLEEP_S 600  // sleep after giving up, then retry with a fresh GPS fix

// ---- Pass planning --------------------------------------------------------
// The prediction no longer answers "when is the next pass" but "which wake is
// worth making". Every pass above MinElev in the next day is listed, passes
// close enough together are served by a single wake, and a wake that cannot
// carry enough messages is not made at all.

// Two passes closer than this are served by one wake instead of two. The
// threshold is what a fresh wake costs, and a wake is dominated by the GPS fix:
// MAX_GPS_TIMEOUT is 150 s on both cards. Staying awake through a shorter gap is
// cheaper than sleeping and re-acquiring. This also subsumes the overlapping
// case, which used to need its own branch.
#define SPP_MERGE_GAP_S 150

// A wake spends its first seconds on the GPS fix before anything can go out, so
// the messages a session yields are (duration - fix) / interval.
#define SPP_GPS_FIX_S 30

// Matias's call, 3 Sep 2026: when planning, look at every pass, because short
// ones stack into a session and then they are worth having - but never wake for
// an isolated pass that can only carry three or four data messages. Five is the
// floor. Note this makes minPassDurationMinute redundant, which is why the
// prediction below no longer filters on duration.
#define SPP_MIN_DATA_MSGS 5

// Room for the pass list. A day above 5 deg gives about 120 passes over the
// whole fleet; above 30 deg, about 50. Static, not on the stack.
#define SPP_MAX_PASSES 140

// Floor used when logging which satellite a message went out to, deliberately
// well below any planning floor. Attribution describes what was really overhead;
// planning decides what is worth waking for, and the two should not share a
// number. A message sent while the satellite sits at 38 deg is better recorded
// as 38 deg than as "nothing there" because the planner happened to run at 45.
#define SPP_ATTRIBUTION_MIN_ELEV 5.0f

// Print the whole day's session table over serial on every prediction, marking
// the one taken and the ones passed over as too small. Serial only, and only
// with SERIAL_DEBUG on, so it costs nothing on a deployed buoy - but comment it
// out once the plan has been eyeballed enough, because it is a lot of lines.
#define SPP_DUMP_PLAN
// How long the peripherals need after their rail comes back before they will
// answer. Measured on the Arribada 9 Aug 2026: it first replied to AT+ID 482 ms
// after power was restored. The 5 ms that used to be here meant the firmware was
// talking to a module that was still booting, so AT+KMAC came back with the boot
// banner instead of +OK and the AT+TX after it was refused with +ERROR=253.
#define SAT_MODULE_BOOT_MS 1000
// Below this the supply cannot fire the Argos power amplifier. Arribada document
// battery power as required for uplink - USB alone is enough to talk to the
// module but not to transmit - and it shows exactly that way: every command is
// answered normally and then AT+TX gets total silence. Measured on the bench,
// 2.1 V with no battery fails every single time and 4.55 V on cells never does.
#define SAT_TX_MIN_SUPPLY_V 3.0f

// Below this the pack is not fitted, rather than flat, and the buoy is running
// on the USB cable. The two cells operate between about 3.6 and 4.2 V and Li-ion
// is destroyed below 3.0, so a working buoy never legitimately reads down here.
// What does read here is the divider floating with no pack in it: 2.1 V, the
// same 2.1 V proven on the bench to fail every transmission while a healthy pack
// sends fine. Same number as SAT_TX_MIN_SUPPLY_V above, for the same physical
// reason - there is nothing supplying the module.
//
// Worth separating from Bat_critlevel because the right response is opposite:
// a flat pack should drop to LOWPWR to survive, a missing pack means somebody is
// working on the bench and dropping to LOWPWR just wastes the session.
#define BAT_ABSENT_V 3.0f
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
// Operating elevation floor, and the value MinElev falls back on when conf.txt
// cannot be read - it had no initialiser at all, which left it at zero and would
// have had the buoy wake for every pass above the horizon.
//
// 25 rather than the 30 used through August. Measured over the 3 Sep AOP: awake
// time per reconstructable image is nearly flat from 20 to 40 deg, so the floor
// buys latency rather than energy. At 25 an image goes out every 1.9 days at
// 24 % duty against 3.0 days at 16 % for 30, for the same energy per image, and
// 25 is still inside the range the reception tests actually covered.
#define stdMinElev 25.0f                                                                                                                                 // SD
#define critMinElev 45.0f                                                                                                                               // SD
#define FORCE_A2_UPLINK_STATUS
