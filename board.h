#pragma once
#include <Arduino.h>

// Which carrier board this firmware is running on, worked out at boot so one
// binary serves both.
//
//   V1 - the original buoy: EzSBC ESP32 with discrete peripherals and Crydom
//        relays. GPS and satellite module share one rail on GPIO13, PB_3 is on
//        GPIO27, the battery is read through a divider on GPIO36.
//   V2 - PopUp_board_cilinder_v9: the GPS has its own load switch on GPIO27,
//        GPIO13 only drives the satellite module, PB_3 moved to GPIO39 and the
//        battery is read from a MAX17048 gauge over I2C.
//
// GPIO27 is the dangerous one: it is a button on a V1 and an output on a V2, so
// nothing may drive it before boardDetect() has run.

enum BoardVersion : uint8_t { BOARD_V1 = 1, BOARD_V2 = 2 };

// Works the board out. Call it first thing in setup(), before any pinMode().
// Only reads pins, except for GPIO13, which is safe to raise on both boards.
BoardVersion boardDetect();

BoardVersion boardVersion();
bool boardIsV2();
const char *boardName();

// One line saying how the board was recognised, for the log.
const char *boardDetectReason();

// PB_3 moved between boards, so it is looked up rather than fixed.
int boardPinPB3();
#define PB_3 boardPinPB3()
