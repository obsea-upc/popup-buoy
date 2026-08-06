#include "power_sleep.h"
#include "sat_module.h"
#include "conf.h"
#include "logging.h"
#include "eeprom_store.h"   // for eepromReadSyncTime
#include "wifi_http.h"      // for wifiShutdown
#include <RTClib.h>
#include <SD.h>
#include <esp_sleep.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern RTC_DS3231 rtcExt;
extern int syncTime;
extern int currentState;

void SleepModeSequence(int8_t sleepingHours, int8_t sleepingMinute, int8_t sleepingSecond, int sleepMode) {
  //Leave the AP cleanly before we vanish (still on SD power, so this can be logged)
  wifiShutdown();
  //Disconnect Peripherals
  ConnectPeripherals(false, GPS_KIM);
  delay(10);
  ConnectPeripherals(false, SD_card);
  //Light Sequence
  lightSequenceSleep();
  //Enter Sleep mode...
  if(sleepMode == 0){
    SerialPrintDebugln("Sleeping relative time");
    goToSleepRTC_rel(sleepingHours, sleepingMinute, sleepingSecond);
  }else{
    SerialPrintDebugln("Sleeping absolute time");
    goToSleepRTC_abs(sleepingHours);
  }

}

void goToSleep(int sleeping_time) {  //no need to turn off pheriperals, already done
  SerialPrintDebugln("Starting Light Sleep Routine");
  delay(10);
  if (sleeping_time == 0){
    sleeping_time=1;
  }
  //End all SD process
    SD.end();
  //Turn off peripherals (except for case 6)
   if (currentState != ST_FRM){
      ConnectPeripherals(false, GPS_KIM);  // turn off power to all devices (not in case &)
      delay(5);
      ConnectPeripherals(false, SD_card);
      delay(5);
    }
  //Gotoleep light
    esp_sleep_enable_timer_wakeup(sleeping_time * uS_TO_S_FACTOR);
    esp_light_sleep_start();
  //Turn on peripherals (except for case 6)
    if (currentState != ST_FRM){
      ConnectPeripherals(true, GPS_KIM);
      // The satellite module needs far longer than the 5 ms this used to allow
      // before it will answer AT commands. Talking to it too early is what made
      // the first transmission after every wake time out while the following
      // ones worked. satModuleWakeUp() polls instead of guessing, so it only
      // costs as long as the module actually takes.
      satModuleWakeUp();
      ConnectPeripherals(true, SD_card);
      delay(5);
    }
  //initialize again SD
    if (!SD.begin()) {
          SerialPrintDebugln("Card Mount Failed");
          return;   //ojo amb aquest return --> posar while?
    }else{
      SerialPrintDebugln("SD card open again");
    }
  delay(10);
}

void goToSleepRTC_rel(int8_t sleepingHours, int8_t sleepingMinute, int8_t sleepingSecond) {

   rtcExt.clearAlarm(1);
  if (!rtcExt.setAlarm1(rtcExt.now() + TimeSpan(0, sleepingHours, sleepingMinute, sleepingSecond), DS3231_A1_Hour)) {
    SerialPrintDebugln("Error, alarm wasn't set!");

  } else {
    //SerialPrintDebug("Alarm will happen in 10 seconds!");
    DateTime now = rtcExt.now();

    // the stored alarm value + mode
    DateTime alarm1 = rtcExt.getAlarm1();
    Ds3231Alarm1Mode alarm1mode = rtcExt.getAlarm1Mode();
    char alarm1Date[20] = "hh:mm:ss";
    alarm1.toString(alarm1Date);
    SerialPrintDebug(" [Alarm1: ");
    SerialPrintDebug(alarm1Date);
    SerialPrintDebug(", Mode: ");
    switch (alarm1mode) {
      case DS3231_A1_PerSecond: SerialPrintDebugln("PerSecond"); break;
      case DS3231_A1_Second: SerialPrintDebugln("Second"); break;
      case DS3231_A1_Minute: SerialPrintDebugln("Minute"); break;
      case DS3231_A1_Hour: SerialPrintDebugln("Hour"); break;
      case DS3231_A1_Date: SerialPrintDebugln("Date"); break;
      case DS3231_A1_Day: SerialPrintDebugln("Day"); break;
    }
  }
  SerialPrintDebugln("going to sleep");
  //SD.end();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // pin for the external RTC
  esp_deep_sleep_start();
}

void goToSleepRTC_abs(int8_t sleepingHours) {
  rtcExt.clearAlarm(1);

  DateTime now = rtcExt.now();
  DateTime alarmTime;
  syncTime = eepromReadSyncTime();

  if (sleepingHours == 24) {
    // Configurar la alarma para las 9:00 AM de hoy si aún no ha pasado; si no, configurar para las 9:00 AM del día siguiente
    if (now.hour() < syncTime) {
      alarmTime = DateTime(now.year(), now.month(), now.day(), syncTime, 0, 0);
    } else {
      alarmTime = DateTime(now.year(), now.month(), now.day(), syncTime, 0, 0) + TimeSpan(1, 0, 0, 0);
    }
  } else {
    // Configurar la alarma para la próxima hora en punto transcurridas las sleepingHours
    alarmTime = DateTime(now.year(), now.month(), now.day(), now.hour(), 0, 0) + TimeSpan(0, sleepingHours, 0, 0);
  }

  if (!rtcExt.setAlarm1(alarmTime, DS3231_A1_Date)) {
    SerialPrintDebugln("Error, alarm wasn't set!");
  } else {
    // Imprimir fecha y hora de la alarma en la misma línea que el modo
    char alarm1Date[20] = "YYYY-MM-DD hh:mm:ss";
    alarmTime.toString(alarm1Date);
    SerialPrintDebug(" [Alarm1: ");
    SerialPrintDebug(alarm1Date);  // Imprimir fecha y hora en la misma línea
    SerialPrintDebugln(", Mode: Date");
  }
  SerialPrintDebugln("going to sleep");
  //SD.end();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  // pin para la alarma del RTC
  esp_deep_sleep_start();
}

void ConnectPeripherals(bool activateRelay, int PRelay) {
  #ifdef DISCONNECT_PHER
    if (activateRelay == true) {
      SerialPrintDebugln("Activating Peripherals in " + String(PRelay));
      digitalWrite(PRelay, HIGH);

    } else {
      SerialPrintDebugln("Deactivating Peripherals in " + String(PRelay));
      digitalWrite(PRelay, LOW);
    }
  #endif
}

void lightSequenceSleep() {
  //slow flash yellow led
  for (int i = 0; i <= 2; i++) {
    digitalWrite(LED_Y, HIGH);
    delay(500);
    digitalWrite(LED_Y, LOW);
    delay(500);
  }

  //fast flash yellow led
  for (int i = 0; i <= 5; i++) {
    digitalWrite(LED_Y, HIGH);
    delay(100);
    digitalWrite(LED_Y, LOW);
    delay(100);
  }
}

// Convert a duration in seconds into hours + minutes + remaining seconds.
static void ChangeSecondsInHoursAndMinutes(int *seconds, int *minutes, int *hours) {
  *hours = *seconds / 3600;           // Conversion en heures
  *minutes = (*seconds % 3600) / 60;  // Conversion en minutes
  *seconds = (*seconds % 3600) % 60;  // Conversion en secondes sans les heures et les minutes
}

// Change to targetState and deep-sleep for the given number of seconds (converts to h/m/s).
void sleepSecondsAndGoTo(int seconds, int targetState) {
  int h = 0, m = 0, s = seconds;
  ChangeSecondsInHoursAndMinutes(&s, &m, &h);
  changeStateTo(targetState);
  writeLogFile("Entering Sleep mode");
  SleepModeSequence(h, m, s, 0);
  delay(10);
}
