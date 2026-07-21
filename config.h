#pragma once

// Read the mission configuration from the SD card (/conf.txt) into the runtime globals
// (idBuoy, timeouts, sleep times, transmission params, MinElev, PWR2/PWR3, ...).
// Called once at setup so parameters can be changed on the SD card without recompiling.
void getInfoFromConfFile();

// Timing / sleep configuration loaded from /conf.txt (defined in config.cpp).
extern int maxWIFITimeout;
extern int sleeptime_s1_h;
extern int sleeptime_s1_m;
extern int sleepTimeWifiAttempt;
extern int sleeptime_errorGPS_s;
extern int sleeptime_errorGPS_recurrent_s;
extern int max_sleep_time_s;
extern int timetransm_GPS_s;
extern int timetransm_GPS_noArg_s;
extern unsigned long maxFRM;
