#include "power_sleep.h"
#include "conf.h"
#include "logging.h"
#include "eeprom_store.h"   // for eepromReadSyncTime
#include "wifi_http.h"      // for wifiShutdown
#include "gps.h"            // for gpsSerialBegin
#include "board.h"
#include "sat_module.h"     // for satModuleReleaseLines
#include <RTClib.h>
#include <SD.h>
#include <esp_sleep.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern RTC_DS3231 rtcExt;
extern int syncTime;
extern int currentState;
extern HardwareSerial gpsSerial;

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
      // Let go of the GPS transmit line before opening the relay. Measured on the
      // bench 9 Aug: with the rail already cut, GPIO2 on its own kept the whole
      // GPS+KIM supply half alive - the ESP32 holds it high as the idle UART
      // level, the current enters the GPS receive pin, crosses that pin's ESD
      // clamp and lands on the shared rail, so the KIM lit up as well. Bisected
      // pin by pin: dropping the KIM ON/OFF and the KIM transmit line changed
      // nothing at all, dropping this one put both modules out. Only this pin is
      // touched, for that reason - the KIM UART is the transmit path and is left
      // alone.
      gpsSerial.end();
      pinMode(TXPin_GPS, OUTPUT);
      digitalWrite(TXPin_GPS, LOW);

      // Same mechanism on the satellite side of a V2, where it matters far more:
      // VKIM is never switched there, GPIO13 only drives ON/OFF, and with GPIO17
      // held high as the idle UART level the KIM stays in Standby at 3.5 mA for
      // the whole sleep - 40/40 alive on the bench, 22 Sep 2026. The next
      // transmission reopens the port. The V1 keeps its measured behaviour: its
      // relay cuts the supply itself and the line only leaked 1/40.
      if (boardIsV2()) satModuleReleaseLines();

      ConnectPeripherals(false, GPS_KIM);  // turn off power to all devices (not in case &)
      delay(5);
      ConnectPeripherals(false, SD_card);
      delay(5);
    }
  //Gotoleep light
    esp_sleep_enable_timer_wakeup(sleeping_time * uS_TO_S_FACTOR);
    // Measure what the sleep actually did. On 8 Aug 2026 the KIM buoy ran a whole
    // day at a 7.9 s cycle instead of 31.6 s because this call kept returning
    // immediately, and there was no way to tell from the logs. A bench probe on
    // the same board later slept correctly every time, so whatever rejects the
    // sleep is a runtime condition - most likely another wakeup source already
    // asserted. Record the duration and the cause so the next occurrence names it.
    const uint32_t sleepEntryMs = millis();
    esp_light_sleep_start();
    const uint32_t sleptMs = millis() - sleepEntryMs;
    const int wakeCause = (int)esp_sleep_get_wakeup_cause();
  //Turn on peripherals (except for case 6)
    if (currentState != ST_FRM){
      ConnectPeripherals(true, GPS_KIM);
      if (boardIsV2()) satModuleRestoreLines();   // at power-on, not at the next send
      delay(5);
      ConnectPeripherals(true, SD_card);
      // Give the satellite module time to boot before anyone talks to it. Now
      // that the rail really does switch off, it restarts on every one of these
      // and it is not ready for 482 ms. With the 5 ms that used to be here, the
      // AT+KMAC sent straight after came back as the module's boot banner rather
      // than +OK, so the profile was never set and the next AT+TX was refused
      // with +ERROR=253 - a whole DM cycle lost per message.
      delay(SAT_MODULE_BOOT_MS);
      // V1: the port has to be reopened here. gpsAcquireData() does not open it
      // there - it is opened once at boot and once in gpsAcquireSatellites,
      // neither of which runs again in DM - so without this the receiver would
      // go silent for the rest of the drift after the first light sleep.
      // V2: the GPS is off between fixes and gpsAcquireData() powers it and
      // opens the port together; opening it here would feed the idle line into
      // an unpowered module.
      if (!boardIsV2()) gpsSerialBegin();
    }
  //initialize again SD
    if (!SD.begin()) {
          SerialPrintDebugln("Card Mount Failed");
          return;   //ojo amb aquest return --> posar while?
    }else{
      SerialPrintDebugln("SD card open again");
    }
  // Logged only when the sleep came up short, so a healthy buoy adds nothing to
  // the SD. Has to happen here: the card is only back online after SD.begin().
    if (sleptMs < (uint32_t)sleeping_time * 900UL) {
      writeLogFile("LIGHT SLEEP SHORT: asked " + String(sleeping_time) + " s, slept "
                   + String(sleptMs) + " ms, wake cause " + String(wakeCause));
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
