#include "board.h"
#include "conf.h"
#include <Wire.h>
#include <Preferences.h>

// Opened on the GPS pins by the main sketch; borrowed here for the NMEA test.
extern HardwareSerial gpsSerial;

static BoardVersion current = BOARD_V1;
static char reason[112] = "not detected yet";

static bool i2cAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// On a V2, GPIO39 carries PB_3 with R18, a 10k pull-up to 3.3 V, so it reads
// high unless the button is held. On a V1 the same pin sits at 0 V. Measured
// 23 Sep 2026 with tools/board_probe, one ESP32 moved between the two boards
// and no battery: V2 200/200 high at 3150 mV, V1 0/200 at about 140 mV.
static bool pin39PulledUp() {
  pinMode(BOARD_DETECT_PIN, INPUT);   // input only, no internal pull exists
  for (int i = 0; i < 20; i++) {
    if (digitalRead(BOARD_DETECT_PIN) == LOW) return false;
    delayMicroseconds(500);
  }
  return true;
}

// On a V1 the GPS hangs off the GPIO13 relay, so raising GPIO13 makes it talk
// within 70 ms. On a V2 GPIO13 only drives the satellite module's ON/OFF, the
// GPS needs GPIO27 and stays silent. Raising GPIO13 is harmless on both.
static bool gpsTalksOnGpio13() {
  pinMode(GPS_KIM, OUTPUT);
  digitalWrite(GPS_KIM, HIGH);
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin_GPS, TXPin_GPS);

  // "$G" rather than a lone '$', so noise on a floating line cannot pass.
  const uint32_t t0 = millis();
  uint8_t sentences = 0;
  bool dollar = false;
  while (millis() - t0 < BOARD_NMEA_LISTEN_MS && sentences < 2) {
    while (gpsSerial.available()) {
      const char c = gpsSerial.read();
      if (dollar && c == 'G') sentences++;
      dollar = (c == '$');
    }
  }

  // Leave both lines low, as goToSleep() does: the idle-high UART line would
  // otherwise feed the GPS through its ESD clamp.
  gpsSerial.end();
  pinMode(TXPin_GPS, OUTPUT);
  digitalWrite(TXPin_GPS, LOW);
  digitalWrite(GPS_KIM, LOW);
  return sentences >= 2;
}

BoardVersion boardDetect() {
  Preferences prefs;
  prefs.begin("board", false);
  const uint8_t stored = prefs.getUChar("ver", 0);
  bool confident = true;

  Wire.begin();

  // Decisive tests first, cheapest first. The gauge only answers with a battery
  // fitted (its VDD is VBAT), so its silence proves nothing; GPIO39 works on the
  // cable as well.
  if (i2cAck(MAX17048_ADDR)) {
    current = BOARD_V2;
    snprintf(reason, sizeof(reason), "battery gauge answered on I2C");
  } else if (pin39PulledUp()) {
    current = BOARD_V2;
    snprintf(reason, sizeof(reason), "GPIO39 pulled up");
  } else if (stored == BOARD_V1) {
    // A V2 only reads GPIO39 low with PB_3 held, and a board that remembers
    // being a V1 has had this ESP32 in a V1 before: skip the GPS test.
    current = BOARD_V1;
    snprintf(reason, sizeof(reason), "GPIO39 low, V1 remembered");
  } else if (gpsTalksOnGpio13()) {
    current = BOARD_V1;
    snprintf(reason, sizeof(reason), "GPIO39 low and the GPS answered on GPIO13");
  } else if (stored == BOARD_V2) {
    // Most likely a V2 with PB_3 held through the boot.
    current = BOARD_V2;
    confident = false;
    snprintf(reason, sizeof(reason), "GPIO39 low and no GPS on GPIO13 (PB_3 held?): V2 remembered");
  } else {
    // Nothing conclusive and nothing remembered. V1 is the side that cannot
    // hurt: run on a V2 it only leaves the GPS unpowered, while V2 handling on
    // a V1 would drive GPIO27 into the PB_3 button.
    current = BOARD_V1;
    confident = false;
    snprintf(reason, sizeof(reason), "nothing conclusive, nothing remembered: assuming V1");
  }

  // Written only when it changes, so the flash sees one write per board swap.
  if (confident && stored != current) prefs.putUChar("ver", current);
  prefs.end();

  return current;
}

BoardVersion boardVersion() { return current; }
bool boardIsV2() { return current == BOARD_V2; }
const char *boardName() { return current == BOARD_V2 ? "V2" : "V1"; }
const char *boardDetectReason() { return reason; }

int boardPinPB3() { return current == BOARD_V2 ? PB_3_V2 : PB_3_V1; }
