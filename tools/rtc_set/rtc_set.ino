// Reads and sets the buoy's DS3231 over USB, in UTC, without going through the
// CONFIG state and its NTP-over-WiFi step. The mission firmware only ever sets
// the clock from NTP, never from GPS, so a clock that has drifted or lost power
// stays wrong through a whole deployment - and the pass planner trusts it.
//
// Protocol:
//   PC  -> GET             ESP -> NOW <unix> <yyyy-mm-ddThh:mm:ss>
//   PC  -> SET <unix>      ESP -> OK <unix> <yyyy-mm-ddThh:mm:ss>   (read back)
//
// Touches nothing but the RTC: the SD and the satellite rail stay off.

#include <Wire.h>
#include <RTClib.h>

#define SD_card 14
#define GPS_KIM 13

RTC_DS3231 rtc;
static bool rtcOk = false;

static void printNow(const char *tag) {
  const DateTime t = rtc.now();
  Serial.printf("%s %lu %04d-%02d-%02dT%02d:%02d:%02d\n", tag, (unsigned long)t.unixtime(),
                t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
}

void setup() {
  Serial.begin(115200);
  pinMode(SD_card, OUTPUT);
  pinMode(GPS_KIM, OUTPUT);
  digitalWrite(SD_card, LOW);
  digitalWrite(GPS_KIM, LOW);

  Serial.println();
  Serial.println("=== RTC SET ===");
  Wire.begin();
  rtcOk = rtc.begin();
  if (!rtcOk) { Serial.println("FAIL rtc-no-responde"); return; }
  if (rtc.lostPower()) Serial.println("AVISO el RTC ha perdido la alimentacion");
  printNow("NOW");
  Serial.println("esperando GET | SET <unix>");
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!rtcOk) { Serial.println("FAIL rtc-no-responde"); return; }

  if (cmd == "GET") {
    printNow("NOW");
  } else if (cmd.startsWith("SET ")) {
    const unsigned long u = strtoul(cmd.substring(4).c_str(), nullptr, 10);
    if (u < 1700000000UL) { Serial.println("FAIL argumentos"); return; }
    rtc.adjust(DateTime(u));  // also clears the oscillator-stopped flag
    printNow("OK");
  }
}
