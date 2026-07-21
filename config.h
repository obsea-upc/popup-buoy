#pragma once

// Read the mission configuration from the SD card (/conf.txt) into the runtime globals
// (idBuoy, timeouts, sleep times, transmission params, MinElev, PWR2/PWR3, ...).
// Called once at setup so parameters can be changed on the SD card without recompiling.
void getInfoFromConfFile();
