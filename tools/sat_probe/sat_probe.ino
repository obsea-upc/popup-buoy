// Read-only probe of whatever sits in the satellite socket, for when the mission
// firmware reports "Satellite module NOT DETECTED". Powers GPIO13, then asks the
// module to identify itself at several baud rates and prints every byte that
// comes back, non-printables in hex, so silence, garbage (wrong baud) and a real
// answer look different.
//
// Queries only. It never sends AT+SAVE_CFG, AT+RCONF=<blob> or AT+SAVE_RCONF:
// those write the module's flash and have destroyed KIM1s before.
//
// Send any line over serial to run the probe again.

#include <HardwareSerial.h>

#define SD_card 14
#define GPS_KIM 13
#define SAT_RX 16
#define SAT_TX 17
#define GPS_RX 4
#define GPS_TX 2

HardwareSerial sat(2);

static const char *QUERIES[] = { "AT+PING=?", "AT+FW=?", "AT+ID=?", "AT+SN=?", "AT+KMAC=?", "AT+RCONF=?", "AT+AFMT=?", "AT+PWR=?" };
static const long BAUDS[] = { 9600, 115200, 4800, 19200, 57600 };

static bool query(const char *cmd, uint32_t windowMs) {
  while (sat.available()) sat.read();
  sat.print(cmd);
  sat.print("\r");
  Serial.printf("  %-12s -> ", cmd);
  bool any = false;
  const uint32_t t0 = millis();
  while (millis() - t0 < windowMs) {
    while (sat.available()) {
      const int c = sat.read();
      any = true;
      if (c == '\r') continue;
      if (c == '\n') { Serial.print(" | "); continue; }
      if (c >= 32 && c < 127) Serial.write((char)c);
      else Serial.printf("<%02X>", c);
    }
    delay(5);
  }
  Serial.println(any ? "" : "(silencio)");
  return any;
}

static void probe() {
  Serial.println("--- corte de alimentacion 2 s, encendido, espera 2 s");
  digitalWrite(GPS_KIM, LOW);
  delay(2000);
  digitalWrite(GPS_KIM, HIGH);
  delay(2000);

  // The GPS hangs off the same GPIO13 rail. NMEA arriving proves the rail is on,
  // so a silent module is then the module or its wiring, not the power switch.
  {
    HardwareSerial gps(1);
    gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    uint32_t t0 = millis(); int n = 0; String first;
    while (millis() - t0 < 2500) { while (gps.available()) { const char c = (char)gps.read(); n++; if (first.length() < 60 && c >= 32 && c < 127) first += c; } delay(5); }
    gps.end();
    Serial.printf("GPS (mismo rail): %d bytes %s\n", n, n ? first.c_str() : "(silencio: el rail GPIO13 no da tension?)");
  }

  for (int pass = 0; pass < 2; pass++) {
  const int rx = pass ? SAT_TX : SAT_RX, tx = pass ? SAT_RX : SAT_TX;
  Serial.printf("=== UART rx=%d tx=%d %s\n", rx, tx, pass ? "(CRUZADO)" : "(normal)");
  for (long baud : BAUDS) {
    if (pass && baud != 9600) break;   // crossed wiring: 9600 is enough to tell
    Serial.printf("baud %ld\n", baud);
    sat.begin(baud, SERIAL_8N1, rx, tx);
    delay(50);
    // Anything the module printed on boot is still in the buffer at 9600.
    if (sat.available()) {
      Serial.print("  (pendiente) -> ");
      while (sat.available()) { const int c = sat.read(); if (c >= 32 && c < 127) Serial.write((char)c); else Serial.printf("<%02X>", c); }
      Serial.println();
    }
    bool answered = false;
    for (const char *q : QUERIES) answered |= query(q, 1500);
    sat.end();
    if (answered && baud == 9600) break;   // the buoy runs at 9600; only sweep if that is silent
  }
  }
  Serial.println("--- fin (envia cualquier linea para repetir)");
}

void setup() {
  Serial.begin(115200);
  pinMode(SD_card, OUTPUT);
  pinMode(GPS_KIM, OUTPUT);
  digitalWrite(SD_card, LOW);
  delay(300);
  Serial.println();
  Serial.println("=== SAT PROBE ===");
  probe();
}

void loop() {
  if (!Serial.available()) return;
  while (Serial.available()) { Serial.read(); delay(2); }
  probe();
}
