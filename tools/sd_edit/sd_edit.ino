// Edits the buoy's SD card over USB: sets a conf.txt key and resets the
// transmission progress, so a run can be reconfigured without opening a sealed
// enclosure or pulling the card.
//
// Set WANT_* below, flash, read the serial output, then flash the mission
// firmware back. The EEPROM state survives, so the buoy resumes where it was.
//
// conf.txt is never truncated in place. The new text goes to a temporary file
// which is read back and compared before the original is replaced, so a write
// that fails half way leaves the working file untouched.

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

#define SD_card 14
#define GPS_KIM 13

// ---- what to change -------------------------------------------------------
struct ConfEdit { const char *key; const char *value; };
static const ConfEdit WANT[] = {
  { "idBuoy",         "2"    },   // the KIM in this hull is 276f235, which is buoy 2
  { "MinElev",        "5"    },   // low-elevation campaign
  { "BAT_CRIT_LEVEL", "3400" },   // 3.4 V
  { "MAX_FRM_TIME_h", "24"   },
};
static const uint8_t WANT_N = sizeof(WANT) / sizeof(WANT[0]);
static const bool WANT_PROGRESS_RESET = true;   // progressFile.txt back to 1:0
// ---------------------------------------------------------------------------

RTC_DS3231 rtcExt;

static void dump(const char *path) {
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("  (%s no existe)\n", path); return; }
  while (f.available()) Serial.write(f.read());
  Serial.println();
  f.close();
}

// Rewrites conf.txt with every key in WANT set to its new value. Returns false
// and leaves the card untouched if anything about the write looks wrong.
static bool setConfKeys() {
  File in = SD.open("/conf.txt", FILE_READ);
  if (!in) { Serial.println("ERROR: no se puede abrir /conf.txt"); return false; }

  SD.remove("/conf.new");
  File out = SD.open("/conf.new", FILE_WRITE);
  if (!out) { Serial.println("ERROR: no se puede crear /conf.new"); in.close(); return false; }

  bool seen[16] = { false };
  size_t lines = 0;

  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.replace("\r", "");
    line.trim();
    if (line.length() == 0) continue;
    lines++;

    int hit = -1;
    for (uint8_t k = 0; k < WANT_N; k++) {
      const size_t keyLen = strlen(WANT[k].key);
      // Match "KEY=" at the start, so a key that is a prefix of another is safe.
      if (line.length() > keyLen && line.startsWith(WANT[k].key) && line.charAt(keyLen) == '=') {
        hit = k;
        break;
      }
    }
    if (hit >= 0) {
      Serial.printf("  %-32s ->  %s=%s\n", line.c_str(), WANT[hit].key, WANT[hit].value);
      out.printf("%s=%s\n", WANT[hit].key, WANT[hit].value);
      seen[hit] = true;
    } else {
      out.println(line);
    }
  }
  in.close();
  out.close();

  for (uint8_t k = 0; k < WANT_N; k++) {
    if (!seen[k]) {
      Serial.printf("ERROR: no se encontro la clave %s, no se toca nada\n", WANT[k].key);
      SD.remove("/conf.new");
      return false;
    }
  }

  // Read the temporary back and check every key really says what it should
  // before anything replaces the file the buoy actually boots from.
  File chk = SD.open("/conf.new", FILE_READ);
  if (!chk) { Serial.println("ERROR: no se puede releer /conf.new"); return false; }
  bool ok[16] = { false };
  size_t chkLines = 0;
  while (chk.available()) {
    String line = chk.readStringUntil('\n');
    line.replace("\r", "");
    line.trim();
    if (line.length() == 0) continue;
    chkLines++;
    for (uint8_t k = 0; k < WANT_N; k++) {
      if (line == String(WANT[k].key) + "=" + String(WANT[k].value)) ok[k] = true;
    }
  }
  chk.close();

  bool allOk = (chkLines == lines);
  for (uint8_t k = 0; k < WANT_N; k++) if (!ok[k]) allOk = false;
  if (!allOk) {
    Serial.printf("ERROR: verificacion fallida (%u lineas de %u), no se sustituye\n",
                  (unsigned)chkLines, (unsigned)lines);
    SD.remove("/conf.new");
    return false;
  }

  SD.remove("/conf.bak");
  SD.rename("/conf.txt", "/conf.bak");     // el original queda guardado
  if (!SD.rename("/conf.new", "/conf.txt")) {
    Serial.println("ERROR: el renombrado fallo; el original sigue en /conf.bak");
    return false;
  }
  Serial.println("  conf.txt sustituido (original en /conf.bak)");
  return true;
}

static bool resetProgress() {
  SD.remove("/progressFile.txt");
  File f = SD.open("/progressFile.txt", FILE_WRITE);
  if (!f) { Serial.println("ERROR: no se puede crear /progressFile.txt"); return false; }
  f.println("1:0");
  f.close();
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(SD_card, OUTPUT);
  pinMode(GPS_KIM, OUTPUT);
  digitalWrite(GPS_KIM, LOW);

  // Power-cycle the card: a reset leaves the rail as the previous sketch left
  // it, and a card already up stays in whatever state it was in.
  digitalWrite(SD_card, LOW);
  delay(600);
  digitalWrite(SD_card, HIGH);
  delay(1500);

  Serial.println();
  Serial.println("=== SD EDIT ===");

  Wire.begin();
  if (rtcExt.begin()) {
    DateTime n = rtcExt.now();
    Serial.printf("RTC: %04d-%02d-%02dT%02d:%02d:%02d UTC   (temp %.2f C)\n",
                  n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second(),
                  rtcExt.getTemperature());
    Serial.printf("RTC perdio la alimentacion alguna vez: %s\n", rtcExt.lostPower() ? "SI" : "no");
  } else {
    Serial.println("ERROR: no se encuentra el RTC");
  }

  bool sdOk = false;
  for (int i = 0; i < 8 && !sdOk; i++) { sdOk = SD.begin(5); if (!sdOk) delay(700); }
  if (!sdOk) { Serial.println("ERROR: SD.begin() fallo"); return; }

  Serial.println("\n--- conf.txt ANTES ---");
  dump("/conf.txt");
  Serial.println("--- progressFile.txt ANTES (ultima linea manda) ---");
  dump("/progressFile.txt");

  Serial.println("\n--- cambios ---");
  const bool confOk = setConfKeys();
  bool progOk = true;
  if (WANT_PROGRESS_RESET) {
    progOk = resetProgress();
    Serial.printf("  progressFile.txt -> 1:0  %s\n", progOk ? "ok" : "FALLO");
  }

  Serial.println("\n--- conf.txt DESPUES ---");
  dump("/conf.txt");
  Serial.println("--- progressFile.txt DESPUES ---");
  dump("/progressFile.txt");

  // Unmount before anything else can reset the board. Leaving the card mounted
  // is how a half-written FAT survives into the next power cycle, and the
  // mission firmware is careful to SD.end() before every sleep for the same
  // reason. Matias lost a card to this on 9 Sep.
  SD.end();
  Serial.println("SD desmontada");

  Serial.printf("\n=== %s ===\n", (confOk && progOk) ? "EDIT COMPLETE" : "EDIT FAILED");
}

void loop() { }
