// Power profile of the buoy electronics, V1 and V2, for the Nordic PPK2.
//
// Runs on its own, with NO USB cable: the PPK2 sits in ampere-meter mode on the
// battery-to-board jumper, and the USB would feed the board from elsewhere. The
// sketch walks through the states below, 30 s each, forever, so every consumer can
// be worked out by difference (GPS = state 5 - state 3, and so on).
//
// Telling the states apart:
//   - At the start of each state, all three LEDs light together for 300 ms, then
//     the YELLOW LED blinks N times, N = the state number. Measure AFTER that.
//   - While a state lasts, the GREEN LED gives a 20 ms tick every 5 s in the awake
//     states (about 0.01 mA of average, negligible). Sleep states stay dark.
//   - Better still: wire PPK2 logic inputs D0/D1/D2 to the LED pins (GPIO33 yellow,
//     GPIO32 red, GPIO0 green) and the state changes are marked on the PPK2 trace.
//
// States (30 s of measurement each, ~4.5 min per loop):
//   1  deep sleep, every rail off                       -> "Sleep" of Table 2
//   2  light sleep, rails off                           -> DM between messages
//   3  CPU awake, rails off                             -> base of every awake phase
//   4  CPU + SD card powered and mounted
//   5  CPU + GPS powered, searching                     (V1: the relay also powers the sat module)
//   6  CPU + satellite module powered, idle             (V1: same relay as 5, so same as 5)
//   7  CPU + satellite module + ONE transmission at ~3 s (1 W on a KIM1)
//   8  CPU + WiFi scanning                              -> permission / release phases
//
// Board is detected from GPIO39 (10k pull-up on a V2, ~0 V on a V1), as the
// firmware does. GPIO27 is only driven on a V2.

#include <Arduino.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

#define LED_R 32
#define LED_Y 33
#define LED_G 0
#define GPS_KIM 13       // V1: GPS + satellite module relay. V2: satellite module ON/OFF
#define SD_card 14
#define GPS_EN_V2 27     // V2 only: GPS load switch (a button on a V1: never driven there)
#define KIM_ONOFF_V1 12  // V1 original KIM1 shield ON/OFF
#define RX_KIM 16
#define TX_KIM 17
#define RX_GPS 4
#define TX_GPS 2
#define SD_CS 5
#define DETECT_PIN 39

#define STATE_S 30
#define N_STATES 8

RTC_DATA_ATTR static int state = 0;   // survives the deep sleep of state 1
static bool v2 = false;
HardwareSerial satSerial(2);
HardwareSerial gpsSerial(1);

static const char *NAMES[N_STATES + 1] = { "",
  "deep sleep, rails off", "light sleep, rails off", "CPU awake, rails off", "CPU + SD",
  "CPU + GPS searching", "CPU + sat module idle", "CPU + sat module + 1 TX", "CPU + WiFi scanning" };

// Red and yellow light with the pin HIGH; the GREEN one is wired the other way
// round (anode to 3.3 V, the firmware "switches it off" with HIGH). Found on the
// first V2 run, 30 Sep 2026: it had been on the whole time, deep sleep included.
static void greenLed(bool on) { digitalWrite(LED_G, on ? LOW : HIGH); }

static void leds(bool r, bool y, bool g) {
  digitalWrite(LED_R, r); digitalWrite(LED_Y, y); greenLed(g);
}

static void announce(int n) {
  leds(1, 1, 1); delay(300); leds(0, 0, 0); delay(400);
  for (int i = 0; i < n; i++) { digitalWrite(LED_Y, HIGH); delay(150); digitalWrite(LED_Y, LOW); delay(250); }
  delay(300);
}

// Everything off, and the lines that could back-feed a dead module held low
// (the idle-high UART levels power a GPS or a KIM through their ESD clamps).
static void allOff() {
  WiFi.mode(WIFI_OFF);
  SD.end();
  gpsSerial.end(); pinMode(TX_GPS, OUTPUT); digitalWrite(TX_GPS, LOW);
  satSerial.end(); pinMode(TX_KIM, OUTPUT); digitalWrite(TX_KIM, LOW);
  digitalWrite(GPS_KIM, LOW);
  digitalWrite(SD_card, LOW);
  if (v2) digitalWrite(GPS_EN_V2, LOW);
  else digitalWrite(KIM_ONOFF_V1, LOW);
}

static void gpsOn() {
  gpsSerial.begin(9600, SERIAL_8N1, RX_GPS, TX_GPS);
  if (v2) digitalWrite(GPS_EN_V2, HIGH);
  else digitalWrite(GPS_KIM, HIGH);          // V1: the shared relay
}

static void satOn() {
  if (!v2) digitalWrite(KIM_ONOFF_V1, HIGH);
  digitalWrite(GPS_KIM, HIGH);
  delay(1000);                               // module boot, as the firmware waits
  satSerial.begin(9600, SERIAL_8N1, RX_KIM, TX_KIM);
}

static String satCmd(const String &cmd, uint32_t waitMs) {
  while (satSerial.available()) satSerial.read();
  satSerial.print(cmd + "\r");         // CR only, as the KIM library sends: AT+PWR fails with CRLF
  String r; const uint32_t t0 = millis();
  while (millis() - t0 < waitMs) { while (satSerial.available()) r += (char)satSerial.read(); delay(5); }
  r.trim();
  Serial.println("  " + cmd + " -> " + r);
  return r;
}

// One transmission, whichever module is fitted. The KIM1 wants the standard format
// and takes 46 hex; the Arribada wants its MAC profile and whole 4-byte words (48).
static void transmitOnce() {
  satCmd("AT+PING=?", 400);            // first command after boot, to have the module listening
  satCmd("AT+PWR=1000", 400);          // KIM1: 1 W as in the field. Arribada: ignored
  satCmd("AT+AFMT=1,16,32", 400);      // KIM1 standard format
  satCmd("AT+KMAC=1", 400);            // Arribada MAC profile
  String r = satCmd("AT+TX=0000000000000000000000000000000000000000000000", 8000);
  if (r.indexOf("OK") < 0) satCmd("AT+TX=000000000000000000000000000000000000000000000000", 8000);
}

static void tickWait(uint32_t ms) {          // wait, with the green tick every 5 s
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if ((millis() - t0) % 5000 < 20) { greenLed(true); delay(20); greenLed(false); }
    if (gpsSerial.available()) while (gpsSerial.available()) gpsSerial.read();   // keep the port drained
    delay(5);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(DETECT_PIN, INPUT);
  int high = 0; for (int i = 0; i < 20; i++) { high += digitalRead(DETECT_PIN); delayMicroseconds(500); }
  v2 = (high == 20);

  gpio_hold_dis(GPIO_NUM_0);                 // released after the deep-sleep hold
  pinMode(LED_R, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_G, OUTPUT);
  pinMode(GPS_KIM, OUTPUT); pinMode(SD_card, OUTPUT);
  if (v2) pinMode(GPS_EN_V2, OUTPUT); else pinMode(KIM_ONOFF_V1, OUTPUT);
  allOff();

  // Coming back from state 1 lands here: carry on with the next one.
  state = state % N_STATES + 1;
  Serial.printf("\n=== power_profile, board %s ===\n", v2 ? "V2" : "V1");
}

void loop() {
  Serial.printf("[%lu s] state %d: %s\n", millis() / 1000, state, NAMES[state]);
  announce(state);
  allOff();

  switch (state) {
    case 1:                                    // deep sleep: the board reboots into setup()
      Serial.flush();
      // In deep sleep the pads are released, and the green LED lit through GPIO0.
      // Hold it at its off level for the sleep (GPIO0 is an RTC pad).
      greenLed(false);
      gpio_hold_en(GPIO_NUM_0);
      gpio_deep_sleep_hold_en();
      esp_sleep_enable_timer_wakeup((uint64_t)STATE_S * 1000000ULL);
      esp_deep_sleep_start();
      break;
    case 2:
      Serial.flush();
      esp_sleep_enable_timer_wakeup((uint64_t)STATE_S * 1000000ULL);
      esp_light_sleep_start();
      break;
    case 3:
      tickWait(STATE_S * 1000UL);
      break;
    case 4:
      digitalWrite(SD_card, HIGH); delay(100);
      Serial.println(SD.begin(SD_CS) ? "  SD mounted" : "  SD mount FAILED");
      tickWait(STATE_S * 1000UL - 100);
      break;
    case 5:
      gpsOn();
      tickWait(STATE_S * 1000UL);
      break;
    case 6:
      satOn();
      tickWait(STATE_S * 1000UL - 1000);
      break;
    case 7: {
      satOn();
      const uint32_t t0 = millis();
      transmitOnce();
      const uint32_t used = millis() - t0 + 1000;
      if (used < STATE_S * 1000UL) tickWait(STATE_S * 1000UL - used);
      break;
    }
    case 8: {
      WiFi.mode(WIFI_STA);
      const uint32_t t0 = millis();
      while (millis() - t0 < STATE_S * 1000UL) { WiFi.scanNetworks(false, true); }
      break;
    }
  }
  allOff();
  state = state % N_STATES + 1;
}
