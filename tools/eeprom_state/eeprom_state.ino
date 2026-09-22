// Shows the buoy's EEPROM and puts the state machine in a chosen state, for a
// board that comes to the bench in whatever state its last run left it (a buoy
// that ended a test in LOWPWR boots straight back into LOWPWR).
//
// Layout mirrors eeprom_store.h: 0 state, 1 coverage state, 2-3 coverage
// duration, 4 GPS-fail counter, 5 WiFi-fail counter, 6 synctime, 7 data-file
// done.
//
// Byte 7 matters as much as byte 0 now. It is set when a buoy finishes sending
// its data file, and it makes the buoy stay in LOWPWR as a drifter whatever the
// battery says. A board that ended its last campaign that way would drop out of
// DM on its first wake of the next one, with a full file in front of it, so
// STATE clears it along with the counters.
//
// Protocol (serial, 115200):
//   PC  -> DUMP        ESP -> EE b0 b1 b2 b3 b4 b5 b6 b7
//   PC  -> STATE <n>   sets byte 0 to n, clears coverage, both fail counters and
//                      the data-done flag (bytes 1-5 and 7), leaves synctime;
//                      ESP -> OK + DUMP line
//   PC  -> DATADONE <0|1>   sets byte 7 on its own, to retire a buoy by hand or
//                           bring one back without touching its state
// Nothing else is touched: SD and satellite rails stay off.

#include <EEPROM.h>

#define EEPROM_SIZE 8
#define EE_ADDR_DATA_DONE 7
#define SD_card 14
#define GPS_KIM 13

static void dump() {
  Serial.print("EE");
  for (int i = 0; i < EEPROM_SIZE; i++) { Serial.print(' '); Serial.print(EEPROM.read(i)); }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(SD_card, OUTPUT);
  pinMode(GPS_KIM, OUTPUT);
  digitalWrite(SD_card, LOW);
  digitalWrite(GPS_KIM, LOW);
  EEPROM.begin(EEPROM_SIZE);
  Serial.println();
  Serial.println("=== EEPROM STATE ===");
  dump();
  Serial.println("esperando DUMP | STATE <n> | DATADONE <0|1>   (4=DM 5=LOWPWR 6=FRM)");
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd == "DUMP") {
    dump();
  } else if (cmd.startsWith("STATE ")) {
    const long n = cmd.substring(6).toInt();
    if (n < 0 || n > 6) { Serial.println("FAIL argumentos"); return; }
    EEPROM.write(0, (uint8_t)n);
    for (int i = 1; i <= 5; i++) EEPROM.write(i, 0);
    EEPROM.write(EE_ADDR_DATA_DONE, 0);
    EEPROM.commit();
    Serial.print("OK ");
    dump();
  } else if (cmd.startsWith("DATADONE ")) {
    const long v = cmd.substring(9).toInt();
    if (v < 0 || v > 1) { Serial.println("FAIL argumentos"); return; }
    EEPROM.write(EE_ADDR_DATA_DONE, (uint8_t)v);
    EEPROM.commit();
    Serial.print("OK ");
    dump();
  }
}
