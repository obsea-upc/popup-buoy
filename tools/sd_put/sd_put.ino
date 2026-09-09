// Writes a text file to the buoy's SD card over USB, so a new data file can be
// loaded without opening a sealed enclosure or pulling the card.
//
// Protocol, line based so it survives the ESP32 stalling on an SD write:
//   PC  -> PUT <path> <lines>
//   ESP -> READY
//   PC  -> one line at a time, waiting for K after each
//   ESP -> K            (per line)
//   ESP -> DONE <lines> <bytes>   or   FAIL <reason>
//
// The upload goes to <path>.tmp and only replaces the target once the line count
// checks out, and whatever was there is kept as <path>.bak. A transfer that dies
// half way leaves the buoy's working file intact.

#include <SPI.h>
#include <SD.h>

#define SD_card 14
#define GPS_KIM 13

static String readLineBlocking(uint32_t timeoutMs) {
  String s;
  const uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\n') return s;
      if (c != '\r') s += c;
    }
  }
  return String("\x01TIMEOUT");
}

static void handlePut(const String &path, long nLines) {
  // Mounted per operation and unmounted at the end, so the card is never left
  // live waiting for a reset - that is what corrupts the FAT.
  bool mounted = false;
  for (int i = 0; i < 8 && !mounted; i++) { mounted = SD.begin(5); if (!mounted) delay(700); }
  if (!mounted) { Serial.println("FAIL sin-tarjeta"); return; }

  const String tmp = path + ".tmp";
  const String bak = path + ".bak";

  // Create the parent directory if the path names one that is not there yet -
  // a formatted card has no /PopUpBuoy_<id>/ and SD.open() will not make it.
  const int slash = path.lastIndexOf('/');
  if (slash > 0) {
    const String dir = path.substring(0, slash);
    if (!SD.exists(dir.c_str())) {
      if (SD.mkdir(dir.c_str())) Serial.printf("creado directorio %s\n", dir.c_str());
      else { Serial.printf("FAIL no-se-puede-crear-directorio %s\n", dir.c_str()); SD.end(); return; }
    }
  }

  SD.remove(tmp);
  File f = SD.open(tmp.c_str(), FILE_WRITE);
  if (!f) { Serial.printf("FAIL no-se-puede-crear %s\n", tmp.c_str()); SD.end(); return; }

  Serial.println("READY");

  long written = 0;
  for (long i = 0; i < nLines; i++) {
    const String line = readLineBlocking(15000);
    if (line.startsWith("\x01")) {
      f.close(); SD.remove(tmp); SD.end();
      Serial.printf("FAIL timeout-en-linea-%ld\n", i + 1);
      return;
    }
    f.print(line);
    f.print('\n');
    written++;
    Serial.println("K");            // el PC espera esto antes de mandar la siguiente
  }
  f.flush();
  f.close();

  // Count what actually landed rather than trusting the write.
  File chk = SD.open(tmp.c_str(), FILE_READ);
  if (!chk) { Serial.println("FAIL no-se-puede-releer"); SD.end(); return; }
  long got = 0;
  while (chk.available()) { if ((char)chk.read() == '\n') got++; }
  const uint32_t size = chk.size();
  chk.close();

  if (got != written) {
    Serial.printf("FAIL releido-%ld-de-%ld\n", got, written);
    SD.remove(tmp);
    SD.end();
    return;
  }

  SD.remove(bak);
  SD.rename(path.c_str(), bak.c_str());      // lo anterior queda guardado
  if (!SD.rename(tmp.c_str(), path.c_str())) {
    Serial.printf("FAIL renombrado; el original sigue en %s\n", bak.c_str());
    SD.end();
    return;
  }

  // Unmount before the reply goes out, so the PC cannot reset the board while
  // the card is still live. That is what leaves a half-written FAT behind.
  SD.end();
  Serial.printf("DONE %ld %lu\n", got, (unsigned long)size);
}

static bool sdOk = false;

void setup() {
  Serial.begin(115200);
  pinMode(SD_card, OUTPUT);
  pinMode(GPS_KIM, OUTPUT);
  digitalWrite(GPS_KIM, LOW);

  // Power-cycle the card rather than just switching it on. A reset leaves the
  // rail as the previous sketch left it, and a card that was already up stays
  // in whatever state it was in - SD.begin() then fails however many times it
  // is retried. This is the same off/on the mission firmware does around every
  // light sleep.
  digitalWrite(SD_card, LOW);
  delay(600);
  digitalWrite(SD_card, HIGH);
  delay(1500);

  Serial.println();
  Serial.println("=== SD PUT ===");

  // CS named explicitly: the same GPIO5 the firmware uses in satellite_spp.cpp.
  for (int i = 0; i < 8 && !sdOk; i++) {
    sdOk = SD.begin(5);
    if (!sdOk) { Serial.printf("SD.begin intento %d fallido\n", i + 1); delay(700); }
  }
  if (!sdOk) { Serial.println("FAIL SD.begin"); return; }

  Serial.printf("SD montada, %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  // Only proving the card is there. handlePut mounts it again for the transfer
  // and unmounts when done, so nothing is left live between commands.
  SD.end();
  Serial.println("esperando PUT <ruta> <lineas>");
}

void loop() {
  if (!Serial.available()) return;
  const String cmd = readLineBlocking(2000);
  if (!cmd.startsWith("PUT ")) return;
  if (!sdOk) { Serial.println("FAIL sin-tarjeta"); return; }

  const int sp = cmd.indexOf(' ', 4);
  if (sp < 0) { Serial.println("FAIL sintaxis"); return; }
  const String path = cmd.substring(4, sp);
  const long n = cmd.substring(sp + 1).toInt();
  if (path.length() == 0 || n <= 0) { Serial.println("FAIL argumentos"); return; }

  handlePut(path, n);
}
