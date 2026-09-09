// Dumps the buoy's SD card over the USB serial port so a test run can be
// filed without pulling the card out of a sealed enclosure.
//
// Prints a directory listing, then every file of interest wrapped in markers
// the PowerShell side splits on, with a byte count so a truncated transfer is
// detectable rather than silent.
//
// Read-only: it never writes to the card. Flash the mission firmware back
// afterwards - the EEPROM state survives, so the buoy carries on where it was.

#include <SPI.h>
#include <SD.h>

#define SD_card 14      // load switch for the card, conf.h
#define GPS_KIM 13      // left LOW: nothing else needs power for this

static const char *FILES[] = {
  "/LogFile.txt",
  "/progressFile.txt",
  "/conf.txt",
  "/AOP.txt",
  "/GPS_track.csv",
};
static const uint8_t NFILES = sizeof(FILES) / sizeof(FILES[0]);

static void listDir(const char *path, uint8_t depth) {
  File dir = SD.open(path);
  if (!dir) return;
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    for (uint8_t i = 0; i < depth; i++) Serial.print("  ");
    if (f.isDirectory()) {
      Serial.printf("[%s]\n", f.name());
      if (depth < 2) listDir(f.name(), depth + 1);
    } else {
      Serial.printf("%-28s %8u bytes\n", f.name(), (unsigned)f.size());
    }
    f.close();
  }
  dir.close();
}

static void dumpFile(const char *path) {
  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.printf("===MISSING=%s===\n", path);
    return;
  }
  const uint32_t size = f.size();
  Serial.printf("===FILE=%s SIZE=%lu===\n", path, (unsigned long)size);

  // Byte at a time through a small buffer: the log can be hundreds of kB and
  // there is no point holding any of it in RAM.
  uint8_t buf[256];
  uint32_t sent = 0;
  while (f.available()) {
    const int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    Serial.write(buf, n);
    sent += n;
  }
  f.close();
  Serial.printf("\n===ENDFILE=%s SENT=%lu===\n", path, (unsigned long)sent);
}

void setup() {
  Serial.begin(115200);

  pinMode(SD_card, OUTPUT);
  pinMode(GPS_KIM, OUTPUT);
  digitalWrite(SD_card, HIGH);     // card on
  digitalWrite(GPS_KIM, LOW);      // radio and GPS stay off

  delay(2500);
  Serial.println();
  Serial.println("===SD DUMP===");

  if (!SD.begin()) {
    Serial.println("===ERROR=SD.begin() failed===");
    return;
  }

  Serial.printf("card type %d, size %llu MB\n\n",
                (int)SD.cardType(), SD.cardSize() / (1024ULL * 1024ULL));

  Serial.println("===LISTING===");
  listDir("/", 0);
  Serial.println("===ENDLISTING===");
  Serial.println();

  for (uint8_t i = 0; i < NFILES; i++) dumpFile(FILES[i]);

  // Unmount before anything can reset the board: a card left mounted is how a
  // half-written FAT survives into the next power cycle.
  SD.end();
  Serial.println("===DUMP COMPLETE===");
}

void loop() { }
