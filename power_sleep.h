#pragma once
#include <Arduino.h>

// Relay/power control for peripherals (e.g. GPS_KIM, SD_card). No-op unless DISCONNECT_PHER is defined.
void ConnectPeripherals(bool activateRelay, int PRelay);

// LED "going to sleep" blink sequence.
void lightSequenceSleep();

// Turn peripherals off, blink, then deep-sleep until the DS3231 alarm.
// sleepMode 0 = relative time; sleepMode 1 = absolute (aligned to synctime).
void SleepModeSequence(int8_t sleepingHours, int8_t sleepingMinute, int8_t sleepingSecond, int sleepMode);

// Deep-sleep helpers (set the DS3231 alarm, then esp_deep_sleep_start).
void goToSleepRTC_rel(int8_t sleepingHours, int8_t sleepingMinute, int8_t sleepingSecond);
void goToSleepRTC_abs(int8_t sleepingHours);

// Light-sleep for a number of seconds, keeping state (toggles peripherals unless in state 6).
void goToSleep(int sleeping_time);

// Change to targetState and deep-sleep for `seconds` seconds (used by the surface states).
void sleepSecondsAndGoTo(int seconds, int targetState);
