// Everything prepare_buoy needs from the board before the mission firmware goes
// on, in one sketch and one serial session: the SD card, the RTC and the EEPROM.
// It replaces sd_put + rtc_set + eeprom_state, which needed three uploads and a
// board reset per file (each reset cost ~20 s of waiting for a silent banner).
//
// Serial at 921600. Commands, one per line:
//   PING                 -> PREP sd=<0|1> rtc=<0|1> lost=<0|1>
//   PUT <path> <lines>   -> READY, then the PC sends the lines in blocks of BLOCK;
//                           the board answers K <n> after each block (n = lines so
//                           far) and finally DONE <lines> <bytes> or FAIL <reason>
//   RTC GET              -> NOW <unix> <iso>
//   RTC SET <unix>       -> OK <unix> <iso>        (clears the lost-power flag)
//   EE DUMP              -> EE b0 .. b7
//   EE STATE <n>         -> OK EE b0 .. b7  (state n; bytes 1-5 and 7 cleared, as eeprom_state)
//
// The PUT keeps sd_put's safety: it writes <path>.tmp, re-reads it, counts the
// lines and only then swaps it in, keeping the old file as <path>.bak. The card is
// mounted per command and unmounted before the answer goes out.
//
// EEPROM layout mirrors eeprom_store.h: 0 state, 1 coverage state, 2-3 coverage
// duration, 4 GPS-fail counter, 5 WiFi-fail counter, 6 synctime, 7 data-file done.

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>

#define SD_card 14
#define GPS_KIM 13
#define SD_CS 5
#define BAUD 921600
#define BLOCK 64            // lines per acknowledgement; must match prepare_buoy.ps1
#define EEPROM_SIZE 8
#define EE_ADDR_DATA_DONE 7

RTC_DS3231 rtc;
static bool sdOk = false, rtcOk = false;

static String readLine(uint32_t timeoutMs) {
  String s;
  const uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\n') return s;
      if (c != '\r') s += c;
    }
    delay(0);
  }
  return String("\x01TIMEOUT");
}

static bool mount() {
  for (int i = 0; i < 5; i++) { if (SD.begin(SD_CS)) return true; delay(300); }
  return false;
}

static void handlePut(const String &path, long nLines) {
  if (!mount()) { Serial.println("FAIL sin-tarjeta"); return; }
  const String tmp = path + ".tmp", bak = path + ".bak";

  // A formatted card has no /PopUpBuoy_<id>/ and SD.open() will not create it.
  const int slash = path.lastIndexOf('/');
  if (slash > 0) {
    const String dir = path.substring(0, slash);
    if (!SD.exists(dir.c_str()) && !SD.mkdir(dir.c_str())) {
      Serial.printf("FAIL no-se-puede-crear-directorio %s\n", dir.c_str()); SD.end(); return;
    }
  }
  SD.remove(tmp);
  File f = SD.open(tmp.c_str(), FILE_WRITE);
  if (!f) { Serial.printf("FAIL no-se-puede-crear %s\n", tmp.c_str()); SD.end(); return; }
  Serial.println("READY");

  long written = 0;
  while (written < nLines) {
    const String line = readLine(15000);
    if (line.startsWith("\x01")) {
      f.close(); SD.remove(tmp); SD.end();
      Serial.printf("FAIL timeout-en-linea-%ld\n", written + 1);
      return;
    }
    f.print(line); f.print('\n');
    written++;
    if (written % BLOCK == 0 || written == nLines) Serial.printf("K %ld\n", written);
  }
  f.flush(); f.close();

  File chk = SD.open(tmp.c_str(), FILE_READ);
  if (!chk) { Serial.println("FAIL no-se-puede-releer"); SD.end(); return; }
  long got = 0;
  uint8_t buf[512];
  while (chk.available()) {
    const int n = chk.read(buf, sizeof buf);
    for (int i = 0; i < n; i++) if (buf[i] == '\n') got++;
  }
  const uint32_t size = chk.size();
  chk.close();
  if (got != written) { Serial.printf("FAIL releido-%ld-de-%ld\n", got, written); SD.remove(tmp); SD.end(); return; }

  SD.remove(bak);
  SD.rename(path.c_str(), bak.c_str());
  if (!SD.rename(tmp.c_str(), path.c_str())) {
    Serial.printf("FAIL renombrado; el original sigue en %s\n", bak.c_str()); SD.end(); return;
  }
  SD.end();
  Serial.printf("DONE %ld %lu\n", got, (unsigned long)size);
}

static void printNow(const char *tag) {
  const DateTime t = rtc.now();
  Serial.printf("%s %lu %04d-%02d-%02dT%02d:%02d:%02d\n", tag, (unsigned long)t.unixtime(),
                t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
}

static void eeDump(const char *prefix) {
  Serial.print(prefix);
  Serial.print("EE");
  for (int i = 0; i < EEPROM_SIZE; i++) { Serial.print(' '); Serial.print(EEPROM.read(i)); }
  Serial.println();
}

void setup() {
  Serial.setRxBufferSize(16384);    // a whole block in flight while the card writes
  Serial.begin(BAUD);
  pinMode(GPS_KIM, OUTPUT); digitalWrite(GPS_KIM, LOW);
  pinMode(SD_card, OUTPUT);

  // Power-cycle the card: a reset leaves the rail as the previous sketch left it,
  // and a card that was already up can refuse SD.begin() however often it is tried.
  digitalWrite(SD_card, LOW); delay(400);
  digitalWrite(SD_card, HIGH); delay(800);
  sdOk = mount();
  if (sdOk) SD.end();

  Wire.begin();
  rtcOk = rtc.begin();
  EEPROM.begin(EEPROM_SIZE);
  Serial.println();
  Serial.println("=== BUOY PREP ===");
}

void loop() {
  if (!Serial.available()) return;
  String cmd = readLine(2000);
  cmd.trim();
  if (cmd.length() == 0 || cmd.startsWith("\x01")) return;

  if (cmd == "PING") {
    Serial.printf("PREP sd=%d rtc=%d lost=%d\n", sdOk, rtcOk, rtcOk && rtc.lostPower());
  } else if (cmd.startsWith("PUT ")) {
    if (!sdOk) { Serial.println("FAIL sin-tarjeta"); return; }
    const int sp = cmd.indexOf(' ', 4);
    if (sp < 0) { Serial.println("FAIL sintaxis"); return; }
    const String path = cmd.substring(4, sp);
    const long n = cmd.substring(sp + 1).toInt();
    if (path.length() == 0 || n < 0) { Serial.println("FAIL argumentos"); return; }
    handlePut(path, n);
  } else if (cmd == "RTC GET") {
    if (!rtcOk) { Serial.println("FAIL rtc-no-responde"); return; }
    printNow("NOW");
  } else if (cmd.startsWith("RTC SET ")) {
    if (!rtcOk) { Serial.println("FAIL rtc-no-responde"); return; }
    const unsigned long u = strtoul(cmd.substring(8).c_str(), nullptr, 10);
    if (u < 1700000000UL) { Serial.println("FAIL argumentos"); return; }
    rtc.adjust(DateTime(u));
    printNow("OK");
  } else if (cmd == "EE DUMP") {
    eeDump("");
  } else if (cmd.startsWith("EE STATE ")) {
    const long n = cmd.substring(9).toInt();
    if (n < 0 || n > 6) { Serial.println("FAIL argumentos"); return; }
    EEPROM.write(0, (uint8_t)n);
    for (int i = 1; i <= 5; i++) EEPROM.write(i, 0);
    EEPROM.write(EE_ADDR_DATA_DONE, 0);
    EEPROM.commit();
    eeDump("OK ");
  } else {
    Serial.println("FAIL comando");
  }
}
