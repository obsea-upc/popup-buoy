// Bring-up console for the V2 board (PopUp_board_cilinder_v9).
//
// One sketch with a serial menu instead of one sketch per subsystem, so the
// board can be walked through without reflashing between tests. Type a letter
// at 115200 baud.
//
// V2 ONLY. It drives GPIO27 as an output, which on a V1 board is the PB_3
// button - driving that high and pressing the button shorts the pin to ground.
// Do not run this on an old buoy.
//
// Nothing here transmits and nothing writes a module's flash.

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <TinyGPS++.h>
#include <esp_sleep.h>

// ---- V2 pin map (conf.h of the mission firmware will follow this) ----
#define LED_R   32
#define LED_Y   33
#define LED_G   0     // strapping pin: fine here, the green LED has Vf 3.1 V
#define PB_1    25
#define PB_2    26
#define PB_3    39    // input only, no internal pull-up: external R18 10k
#define EN_SAT  13    // KIM ON/OFF, or the SY6280 that makes VARR
#define EN_SD   14
#define EN_GPS  27
#define SD_CS    5
#define GPS_RX   4    // GPS TXD -> ESP32
#define GPS_TX   2
#define SAT_RX  16
#define SAT_TX  17
#define RTC_INT 34    // DS3231 INT/SQW, the deep-sleep wake line

#define MAX17048_ADDR 0x36
#define DS3231_ADDR   0x68

RTC_DS3231 rtc;

// ---------------------------------------------------------------- helpers

static uint16_t gaugeRead(uint8_t reg, bool *ok) {
  Wire.beginTransmission(MAX17048_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { *ok = false; return 0; }
  if (Wire.requestFrom((int)MAX17048_ADDR, 2) != 2) { *ok = false; return 0; }
  const uint16_t hi = Wire.read();
  const uint16_t lo = Wire.read();
  *ok = true;
  return (hi << 8) | lo;
}

// ---------------------------------------------------------------- tests

static void testLeds() {
  Serial.println("LEDs: rojo, amarillo, verde, y los tres juntos.");
  const int pins[3] = { LED_R, LED_Y, LED_G };
  const char *names[3] = { "ROJO (32)", "AMARILLO (33)", "VERDE (0)" };
  for (int i = 0; i < 3; i++) {
    Serial.printf("  %s\n", names[i]);
    digitalWrite(pins[i], HIGH); delay(1200); digitalWrite(pins[i], LOW);
    delay(300);
  }
  Serial.println("  los tres");
  for (int i = 0; i < 3; i++) digitalWrite(pins[i], HIGH);
  delay(1500);
  for (int i = 0; i < 3; i++) digitalWrite(pins[i], LOW);
  Serial.println("  -> los colores deben coincidir con lo anunciado.");
}

static void testButtons() {
  Serial.println("Botones: pulsa PB_1, PB_2 y PB_3. Cualquier tecla para salir.");
  Serial.println("  (en reposo los tres leen 1; pulsado leen 0)");
  int last1 = -1, last2 = -1, last3 = -1;
  while (!Serial.available()) {
    const int b1 = digitalRead(PB_1), b2 = digitalRead(PB_2), b3 = digitalRead(PB_3);
    if (b1 != last1 || b2 != last2 || b3 != last3) {
      Serial.printf("  PB_1(25)=%d  PB_2(26)=%d  PB_3(39)=%d\n", b1, b2, b3);
      last1 = b1; last2 = b2; last3 = b3;
    }
    delay(30);
  }
  while (Serial.available()) Serial.read();
}

static void testI2C() {
  Serial.println("Escaneo I2C:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", addr);
      if (addr == DS3231_ADDR)        Serial.print("  <- DS3231 (RTC)");
      else if (addr == MAX17048_ADDR) Serial.print("  <- MAX17048 (medidor de bateria)");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("  nada. Sin 3.3_BIS no hay pull-ups ni RTC.");
  Serial.println("  -> se esperan 0x68 siempre, y 0x36 solo si hay bateria en VBAT.");
}

static void testGauge() {
  bool ok = false;
  const uint16_t ver = gaugeRead(0x08, &ok);
  if (!ok) {
    Serial.println("MAX17048 no responde.");
    Serial.println("  Normal si no hay bateria: su VDD es VBAT, no 3.3.");
    return;
  }
  Serial.printf("MAX17048 VERSION = 0x%04X\n", ver);

  const uint16_t vcell = gaugeRead(0x02, &ok);
  if (ok) Serial.printf("  VCELL = %.3f V   (comparar con el multimetro en VBAT)\n",
                        vcell * 0.000078125f);
  const uint16_t soc = gaugeRead(0x04, &ok);
  if (ok) Serial.printf("  SOC   = %.1f %%\n", soc / 256.0f);
  Serial.println("  -> si VCELL coincide con VBAT, el arreglo VDD->VBAT es correcto.");
}

static void testRTC() {
  if (!rtc.begin()) { Serial.println("DS3231 no responde."); return; }
  const DateTime now = rtc.now();
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  Serial.printf("DS3231: %s   OSF(perdio hora)=%s   temp=%.2f C\n",
                buf, rtc.lostPower() ? "SI" : "no", rtc.getTemperature());
  Serial.printf("  INT/SQW en GPIO34 = %d (debe ser 1 en reposo, con R33 de 10k)\n",
                digitalRead(RTC_INT));
}

// Alarm in 10 s, then watch GPIO34 fall. This is the deep-sleep wake line.
static void testAlarm() {
  if (!rtc.begin()) { Serial.println("DS3231 no responde."); return; }
  rtc.disable32K();
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  if (!rtc.setAlarm1(rtc.now() + TimeSpan(0, 0, 0, 10), DS3231_A1_Second)) {
    Serial.println("No se pudo programar la alarma.");
    return;
  }
  Serial.println("Alarma en 10 s. Vigilando GPIO34...");
  const uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    if (digitalRead(RTC_INT) == 0) {
      Serial.printf("  GPIO34 ha bajado a los %lu ms -> el despertar por RTC funciona.\n",
                    (unsigned long)(millis() - t0));
      rtc.clearAlarm(1);
      return;
    }
    delay(20);
  }
  Serial.println("  GPIO34 NO ha bajado. Mirar R33 y la pista INT/SQW.");
}

// Each load switch on its own, 30 s, with a colour LED lit alongside so the
// board says out loud which rail is up while the meter is on it.
#define SWITCH_ON_MS 30000

static void testSwitches() {
  struct { int pin; int led; const char *name; const char *rail; const char *colour; } sw[3] = {
    { EN_SD,  LED_G, "SD  (GPIO14)", "3.3_REG_SD",                   "VERDE"    },
    { EN_GPS, LED_Y, "GPS (GPIO27)", "3.3_REG_GPS",                  "AMARILLO" },
    { EN_SAT, LED_R, "SAT (GPIO13)", "3.3_REG_ARR / ON_OFF del KIM", "ROJO"     },
  };
  for (int i = 0; i < 3; i++) {
    Serial.printf("%s ON  + LED %s -> mide %s durante %d s\n",
                  sw[i].name, sw[i].colour, sw[i].rail, SWITCH_ON_MS / 1000);
    digitalWrite(sw[i].pin, HIGH);
    digitalWrite(sw[i].led, HIGH);
    delay(SWITCH_ON_MS);
    digitalWrite(sw[i].pin, LOW);
    digitalWrite(sw[i].led, LOW);
    Serial.printf("%s OFF -> debe caer a 0 V\n", sw[i].name);
    delay(3000);
  }
  Serial.println("  -> con los tres en bajo, los pull-down de 100k los mantienen apagados.");
}

static void testSD() {
  Serial.println("SD: alimentando el rail (GPIO14)...");
  digitalWrite(EN_SD, HIGH);
  delay(50);
  if (!SD.begin(SD_CS)) {
    Serial.println("  SD.begin() ha fallado.");
    digitalWrite(EN_SD, LOW);
    return;
  }
  Serial.printf("  montada, %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  File f = SD.open("/v2_test.txt", FILE_WRITE);
  if (f) { f.println("v2 bring-up"); f.close(); Serial.println("  escritura OK"); }
  else Serial.println("  no se ha podido escribir");
  File r = SD.open("/v2_test.txt");
  if (r) { Serial.printf("  lectura: %s", r.readString().c_str()); r.close(); }

  // Clean power-down: release the SPI pins before cutting the rail, or the
  // ESP32 feeds the powered-off card through them.
  SD.end();
  SPI.end();
  pinMode(23, INPUT); pinMode(18, INPUT); pinMode(SD_CS, INPUT); pinMode(19, INPUT);
  digitalWrite(EN_SD, LOW);
  Serial.println("  rail cortado tras soltar el SPI.");
}

// The receiver answered the first run with "More than 100 frame errors, UART RX
// was disabled": while its rail comes up, GPIO2 is still an unconfigured input,
// so the GPS receive pin floats and the module reads the noise as broken
// characters until it gives up and shuts its receiver. Anything sent after that
// is ignored - which is exactly the UBX configuration we want to be able to do.
// So the line is driven to the UART idle level BEFORE the module has power.
static void gpsPowerUp() {
  pinMode(GPS_TX, OUTPUT);
  digitalWrite(GPS_TX, HIGH);      // UART idle, before the module can see it
  digitalWrite(EN_GPS, HIGH);
  delay(300);                      // let the rail settle and the module boot
}

// Mirror image: let go of the transmit line before cutting the rail, or the
// ESP32 feeds the powered-off module through its receive pin's ESD clamp.
static void gpsPowerDown(HardwareSerial &gps) {
  gps.end();
  pinMode(GPS_TX, OUTPUT);
  digitalWrite(GPS_TX, LOW);
  digitalWrite(EN_GPS, LOW);
}

static void testGPS() {
  Serial.println("GPS: alimentando el rail (GPIO27) y escuchando 10 s a 9600...");
  gpsPowerUp();
  HardwareSerial gps(1);
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  const uint32_t t0 = millis();
  int n = 0;
  while (millis() - t0 < 10000) {
    while (gps.available()) { Serial.write(gps.read()); n++; }
    delay(5);
  }
  gpsPowerDown(gps);
  Serial.printf("\n  %d bytes. Sin fix en interior es normal; lo que importa es que hable.\n", n);
  Serial.println("  -> si NO aparece 'frame errors', su UART de entrada sigue viva.");
}

// ---- UBX: can this receiver actually be configured? -----------------------
// The V1 modules were clones that swallowed every command. This one says it is
// a genuine NEO-M8N (MOD=NEO-M8N-0, PROTVER=18.00), so UBX should work - but
// only if its receiver has not shut itself down over framing errors.

static void ubxSend(HardwareSerial &gps, uint8_t cls, uint8_t id,
                    const uint8_t *payload, uint16_t len) {
  uint8_t head[6] = { 0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < 6; i++) { ckA += head[i]; ckB += ckA; }
  for (uint16_t i = 0; i < len; i++) { ckA += payload[i]; ckB += ckA; }
  gps.write(head, 6);
  if (len) gps.write(payload, len);
  gps.write(ckA);
  gps.write(ckB);
  gps.flush();
}

// Waits for a UBX frame of the given class/id. Returns its payload length, or
// -1 on timeout. NMEA keeps flowing meanwhile, so the sync bytes are hunted.
static int ubxWait(HardwareSerial &gps, uint8_t cls, uint8_t id,
                   uint8_t *out, size_t outSize, uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  int state = 0;
  uint8_t gotCls = 0, gotId = 0;
  uint16_t len = 0, idx = 0;

  while (millis() - t0 < timeoutMs) {
    while (gps.available()) {
      const uint8_t c = gps.read();
      switch (state) {
        case 0: state = (c == 0xB5) ? 1 : 0; break;
        case 1: state = (c == 0x62) ? 2 : 0; break;
        case 2: gotCls = c; state = 3; break;
        case 3: gotId = c;  state = 4; break;
        case 4: len = c;    state = 5; break;
        case 5: len |= (uint16_t)c << 8; idx = 0;
                state = (len == 0) ? 7 : 6; break;
        case 6: if (idx < outSize) out[idx] = c;
                if (++idx >= len) state = 7;
                break;
        case 7: state = 8; break;            // checksum A, not verified here
        case 8:
          if (gotCls == cls && gotId == id) return (int)len;
          state = 0;
          break;
      }
    }
    delay(2);
  }
  return -1;
}

static void testUBX() {
  Serial.println("UBX: se puede configurar este receptor?");
  gpsPowerUp();
  HardwareSerial gps(1);
  gps.setRxBufferSize(1024);
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(500);
  while (gps.available()) gps.read();

  uint8_t buf[64];

  // 1. Poll CFG-NAV5 (empty payload). A reply proves the receive path works.
  Serial.println("  [1] consulta de CFG-NAV5...");
  ubxSend(gps, 0x06, 0x24, nullptr, 0);
  int n = ubxWait(gps, 0x06, 0x24, buf, sizeof(buf), 3000);
  if (n < 0) {
    Serial.println("      sin respuesta -> su UART de entrada sigue cerrada o no llega la senal.");
    gpsPowerDown(gps);
    return;
  }
  Serial.printf("      responde, %d bytes. dynModel=%u  fixMode=%u\n", n, buf[2], buf[3]);

  // 2. Set the marine dynamic model, in RAM only (no CFG-CFG, nothing saved).
  Serial.println("  [2] enviando dynModel=5 (mar) + fixMode=3 (auto)...");
  uint8_t nav5[36] = { 0 };
  nav5[0] = 0x05; nav5[1] = 0x00;        // mask: dyn + fixMode
  nav5[2] = 5;                           // dynModel 5 = Sea
  nav5[3] = 3;                           // fixMode 3 = auto 2D/3D
  ubxSend(gps, 0x06, 0x24, nav5, sizeof(nav5));
  n = ubxWait(gps, 0x05, 0x01, buf, sizeof(buf), 3000);   // ACK-ACK
  Serial.println(n >= 0 ? "      ACK recibido." : "      sin ACK (mirar tambien ACK-NAK).");

  // 3. Read it back.
  Serial.println("  [3] releyendo...");
  ubxSend(gps, 0x06, 0x24, nullptr, 0);
  n = ubxWait(gps, 0x06, 0x24, buf, sizeof(buf), 3000);
  if (n < 0) Serial.println("      sin respuesta.");
  else {
    Serial.printf("      dynModel=%u  fixMode=%u  %s\n", buf[2], buf[3],
                  buf[2] == 5 ? "<- CONFIGURACION ACEPTADA" : "<- no ha cambiado");
  }

  gpsPowerDown(gps);
}

// Reads the module's TX line as a passive "is it powered?" probe.
static void testSatRail() {
  Serial.println("Zocalo satelite: el modulo se queda sin tension de verdad?");
  pinMode(SAT_RX, INPUT);
  pinMode(SAT_TX, INPUT);
  const char *phase[3] = { "EN alto", "EN bajo, TX suelto", "EN bajo, TX forzado alto" };
  for (int p = 0; p < 3; p++) {
    if (p == 0) { digitalWrite(EN_SAT, HIGH); pinMode(SAT_TX, INPUT); }
    if (p == 1) { digitalWrite(EN_SAT, LOW);  pinMode(SAT_TX, INPUT); }
    if (p == 2) { pinMode(SAT_TX, OUTPUT); digitalWrite(SAT_TX, HIGH); }
    delay(1500);
    int high = 0;
    for (int i = 0; i < 40; i++) { high += digitalRead(SAT_RX); delay(5); }
    Serial.printf("  %-26s GPIO16 alto %2d/40 -> %s\n", phase[p], high,
                  high > 30 ? "ALIMENTADO" : (high < 5 ? "apagado" : "intermedio"));
  }
  pinMode(SAT_TX, INPUT);
  digitalWrite(EN_SAT, LOW);
}

// The scan finds an undocumented device at 0x0B on this board. The MAX17048
// datasheet fixes its address ("The 7-bit slave address is fixed to 0x6C
// (write)/0x6D (read)", i.e. 0x36), the DS3231 only answers at 0x68, and
// nothing else hangs off SDA/SCL in the netlist. Two candidates: the same
// MAX17048 die answering on a second address, or a false ACK from a marginal
// bus. Reading the same registers from both addresses tells them apart - if
// 0x0B mirrors 0x36 it is one chip, if it NACKs or returns rubbish it is noise.
static void testGhost() {
  const uint8_t addrs[2] = { 0x36, 0x0B };
  const uint8_t regs[4]  = { 0x02, 0x04, 0x08, 0x0C };   // VCELL, SOC, VERSION, CONFIG
  const char *names[4]   = { "VCELL  ", "SOC    ", "VERSION", "CONFIG " };

  for (int r = 0; r < 4; r++) {
    Serial.printf("  %s :", names[r]);
    for (int a = 0; a < 2; a++) {
      bool ok = false;
      Wire.beginTransmission(addrs[a]);
      Wire.write(regs[r]);
      if (Wire.endTransmission(false) == 0 &&
          Wire.requestFrom((int)addrs[a], 2) == 2) {
        const uint16_t hi = Wire.read(), lo = Wire.read();
        Serial.printf("   0x%02X -> 0x%04X", addrs[a], (uint16_t)((hi << 8) | lo));
        ok = true;
      }
      if (!ok) Serial.printf("   0x%02X -> (sin respuesta)", addrs[a]);
    }
    Serial.println();
  }
  Serial.println("  -> valores iguales = un solo chip en dos direcciones.");
  Serial.println("     sin respuesta en 0x0B = ACK falso, bus marginal.");

  // Repeat the scan a few times: a marginal bus is not consistent.
  Serial.print("  repitiendo el escaneo 5 veces, 0x0B aparece en: ");
  int hits = 0;
  for (int i = 0; i < 5; i++) {
    Wire.beginTransmission(0x0B);
    if (Wire.endTransmission() == 0) hits++;
    delay(20);
  }
  Serial.printf("%d de 5\n", hits);
}

// ---- GPS acquisition, and what the coin cell is actually for --------------
// CRBAT feeds the RTC's VBAT and, through D5, the GPS V_BCKP. Its job on the
// GPS side is to keep the ephemeris and the clock in battery-backed RAM while
// the rail is off, so the next power-up is a warm start instead of a cold one.
// The buoy cuts the GPS rail between every DM message, so this is worth real
// money: a cold start is 30-40 s of awake time, every single wake.
//
// So the test is two fixes: one from however the module is now, then a power
// cycle and a second one. If the coin cell is doing its job, the second is
// much faster.

// Waits for a fix, printing progress. Returns time to fix in ms, or -1.
static long waitForFix(HardwareSerial &ser, TinyGPSPlus &nmea, uint32_t limitMs) {
  const uint32_t t0 = millis();
  uint32_t lastReport = 0;
  char line[100];
  size_t li = 0;

  while (millis() - t0 < limitMs) {
    while (ser.available()) {
      const char c = ser.read();
      nmea.encode(c);

      // Echo only the $GNTXT lines: that is where ANTSTATUS lives.
      if (c == '\n') {
        line[li] = '\0';
        if (strstr(line, "TXT") && strstr(line, "ANT")) Serial.printf("    %s\n", line);
        li = 0;
      } else if (c != '\r' && li < sizeof(line) - 1) {
        line[li++] = c;
      }
    }

    if (nmea.location.isValid() && nmea.location.age() < 2000 &&
        nmea.satellites.isValid() && nmea.satellites.value() > 0) {
      return (long)(millis() - t0);
    }

    if (millis() - lastReport >= 10000) {
      lastReport = millis();
      Serial.printf("    %3lu s: satelites=%lu  hdop=%s  chars=%lu\n",
                    (unsigned long)((millis() - t0) / 1000),
                    nmea.satellites.isValid() ? (unsigned long)nmea.satellites.value() : 0UL,
                    nmea.hdop.isValid() ? String(nmea.hdop.hdop(), 1).c_str() : "-",
                    (unsigned long)nmea.charsProcessed());
    }
    delay(5);
  }
  return -1;
}

static void reportFix(TinyGPSPlus &nmea, long ms) {
  if (ms < 0) { Serial.println("    sin fix dentro del tiempo dado."); return; }
  Serial.printf("    FIX en %ld,%03ld s  |  %.6f, %.6f  |  satelites=%lu  hdop=%s\n",
                ms / 1000, ms % 1000,
                nmea.location.lat(), nmea.location.lng(),
                (unsigned long)nmea.satellites.value(),
                nmea.hdop.isValid() ? String(nmea.hdop.hdop(), 1).c_str() : "-");
  if (nmea.date.isValid() && nmea.time.isValid()) {
    Serial.printf("    hora GPS (UTC): %04d-%02d-%02d %02d:%02d:%02d\n",
                  nmea.date.year(), nmea.date.month(), nmea.date.day(),
                  nmea.time.hour(), nmea.time.minute(), nmea.time.second());
  }
}

static void testFix() {
  Serial.println("GPS: adquisicion con antena. Paciencia, esto tarda.");

  gpsPowerUp();
  HardwareSerial gpsSer(1);
  gpsSer.setRxBufferSize(1024);
  gpsSer.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  TinyGPSPlus nmea;
  Serial.println("  [1] primer fix, hasta 240 s:");
  long t1 = waitForFix(gpsSer, nmea, 240000);
  reportFix(nmea, t1);
  if (t1 < 0) { gpsPowerDown(gpsSer); return; }

  // Now the point of the exercise: cut the rail, leave the coin cell holding
  // V_BCKP, and see how fast it comes back.
  Serial.println("  [2] cortando el rail 15 s (la pila mantiene el V_BCKP)...");
  gpsPowerDown(gpsSer);
  delay(15000);

  gpsPowerUp();
  gpsSer.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  TinyGPSPlus nmea2;
  Serial.println("  [3] segundo fix, hasta 120 s:");
  long t2 = waitForFix(gpsSer, nmea2, 120000);
  reportFix(nmea2, t2);

  Serial.println();
  if (t2 >= 0 && t1 > 0) {
    Serial.printf("  primer fix %ld s, segundo %ld s.\n", t1 / 1000, t2 / 1000);
    if (t2 * 3 < t1 || t2 < 15000)
      Serial.println("  -> arranque en caliente: la pila de boton esta haciendo su trabajo.");
    else
      Serial.println("  -> sin mejora clara: mirar D5 y el V_BCKP, o repetir con mas cielo.");
  }
  gpsPowerDown(gpsSer);
}

// Sets the DS3231 from a line typed after the command: "AAAA-MM-DD HH:MM:SS".
static void setRTC() {
  Serial.println("Hora UTC en formato AAAA-MM-DD HH:MM:SS (30 s para enviarla):");
  const uint32_t t0 = millis();
  String s;
  while (millis() - t0 < 30000) {
    while (Serial.available()) {
      const char c = Serial.read();
      if (c == '\n' || c == '\r') { if (s.length() >= 19) goto done; }
      else s += c;
    }
    delay(10);
  }
done:
  s.trim();
  if (s.length() < 19) { Serial.println("  nada valido recibido."); return; }

  const int Y = s.substring(0, 4).toInt(),  Mo = s.substring(5, 7).toInt();
  const int D = s.substring(8, 10).toInt(), H  = s.substring(11, 13).toInt();
  const int Mi = s.substring(14, 16).toInt(), S = s.substring(17, 19).toInt();
  if (Y < 2025 || Mo < 1 || Mo > 12 || D < 1 || D > 31) {
    Serial.printf("  no interpreto '%s'\n", s.c_str());
    return;
  }
  if (!rtc.begin()) { Serial.println("  DS3231 no responde."); return; }
  rtc.adjust(DateTime(Y, Mo, D, H, Mi, S));
  const DateTime now = rtc.now();
  Serial.printf("  puesto en hora: %04d-%02d-%02d %02d:%02d:%02d   OSF=%s\n",
                now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(),
                rtc.lostPower() ? "SI" : "no");
}

// ---- How fast can this receiver be made to fix? ---------------------------
// Three runs, each from a forced COLD start so they are comparable. Without
// the cold start the second run is always fast, because the coin cell kept the
// ephemeris, and the measurement would say nothing.
//
//   A  cold, no help                          <- baseline
//   B  cold + time from the RTC + last position
//   C  cold + the same aiding + 2D fix mode
//
// Aiding uses UBX-MGA (Multiple GNSS Assistance), which is the M8 family's
// mechanism; the older UBX-AID is deprecated on these parts. If the module
// ignored it we would simply see no improvement between A and B.

// CFG-RST, navBbrMask 0xFFFF = cold start, resetMode 0x02 = software, GNSS only.
// It never answers: the receiver is resetting. Nothing in flash is touched.
static void gpsColdStart(HardwareSerial &gps) {
  const uint8_t payload[4] = { 0xFF, 0xFF, 0x02, 0x00 };
  ubxSend(gps, 0x06, 0x04, payload, sizeof(payload));
  delay(2000);
  while (gps.available()) gps.read();
}

static void put32(uint8_t *p, int32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

// UBX-MGA-INI-TIME_UTC: "it is this time, to within tAccS seconds".
static void mgaIniTime(HardwareSerial &gps, const DateTime &t, uint16_t accSeconds) {
  uint8_t p[24] = { 0 };
  p[0] = 0x10;                       // type: UTC time
  p[1] = 0x00;                       // version
  p[2] = 0x00;                       // ref: time is valid on receipt of this message
  p[3] = 0x80;                       // leap seconds unknown
  p[4] = t.year() & 0xFF; p[5] = (t.year() >> 8) & 0xFF;
  p[6] = t.month(); p[7] = t.day();
  p[8] = t.hour();  p[9] = t.minute(); p[10] = t.second();
  p[16] = accSeconds & 0xFF; p[17] = (accSeconds >> 8) & 0xFF;
  ubxSend(gps, 0x13, 0x40, p, sizeof(p));
}

// UBX-MGA-INI-POS_LLH: "you are about here, to within posAcc metres".
static void mgaIniPos(HardwareSerial &gps, double lat, double lon, uint32_t accMetres) {
  uint8_t p[20] = { 0 };
  p[0] = 0x01;                       // type: position, lat/lon/height
  p[1] = 0x00;                       // version
  put32(p + 4,  (int32_t)llround(lat * 1e7));
  put32(p + 8,  (int32_t)llround(lon * 1e7));
  put32(p + 12, 0);                  // altitude, cm
  put32(p + 16, (int32_t)(accMetres * 100UL));
  ubxSend(gps, 0x13, 0x40, p, sizeof(p));
}

// CFG-NAV5. fixMode 1 = 2D only (needs 3 satellites, not 4), 3 = auto.
static void setNav5(HardwareSerial &gps, uint8_t dynModel, uint8_t fixMode) {
  uint8_t p[36] = { 0 };
  p[0] = 0x05; p[1] = 0x00;          // mask: dyn + fixMode
  p[2] = dynModel;
  p[3] = fixMode;
  ubxSend(gps, 0x06, 0x24, p, sizeof(p));
}

// Last known position, as the firmware would keep it in RTC memory.
#define AID_LAT  41.3822
#define AID_LON   2.1848

static long oneRun(const char *label, bool aid, uint8_t fixMode, uint32_t limitMs) {
  Serial.printf("  --- %s\n", label);

  gpsPowerUp();
  HardwareSerial gpsSer(1);
  gpsSer.setRxBufferSize(1024);
  gpsSer.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(600);

  gpsColdStart(gpsSer);
  gpsSer.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(300);

  setNav5(gpsSer, 5, fixMode);       // marine dynamic model in every run
  delay(150);

  if (aid) {
    if (rtc.begin()) {
      const DateTime now = rtc.now();
      mgaIniTime(gpsSer, now, 2);
      Serial.printf("      hora inyectada desde el RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                    now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    }
    mgaIniPos(gpsSer, AID_LAT, AID_LON, 10000);   // within 10 km
    Serial.printf("      posicion inyectada: %.4f, %.4f (+-10 km)\n", AID_LAT, AID_LON);
  }

  TinyGPSPlus nmea;
  const long ms = waitForFix(gpsSer, nmea, limitMs);
  reportFix(nmea, ms);
  gpsPowerDown(gpsSer);
  delay(3000);
  return ms;
}

static void testGpsSpeed() {
  Serial.println("GPS: cuanto se puede acelerar el fix? Tres arranques en frio.");
  Serial.println("  Ojo: en una ventana los numeros son ruidosos. Sirven para comparar,");
  Serial.println("  no como valor absoluto.");

  const long a = oneRun("A: frio, sin ayuda, 3D auto",        false, 3, 180000);
  const long b = oneRun("B: frio + hora y posicion",          true,  3, 180000);
  const long c = oneRun("C: frio + hora y posicion + 2D",     true,  1, 180000);

  Serial.println("\n  === RESUMEN ===");
  Serial.printf("  A  sin ayuda          : %s\n", a < 0 ? "sin fix" : (String(a / 1000.0, 1) + " s").c_str());
  Serial.printf("  B  + hora y posicion  : %s\n", b < 0 ? "sin fix" : (String(b / 1000.0, 1) + " s").c_str());
  Serial.printf("  C  + 2D               : %s\n", c < 0 ? "sin fix" : (String(c / 1000.0, 1) + " s").c_str());
}

// ---- AssistNow Autonomous ------------------------------------------------
// The NEO-M8N predicts its own ephemeris for up to six days from what it has
// already received, with no network and no files (datasheet 1.6.3). That would
// turn a wake with a gap longer than the ~2 h ephemeris validity from a ~30 s
// start into a fast one. It is one configuration bit - but the datasheet says
// the predictions come from "capturing strategic ephemeris data at specific
// times of the day", and this buoy powers its GPS for seconds at a time. So the
// question is not whether it can be enabled; it is whether the receiver ever
// gets the chance to build anything.
//
// CFG-NAVX5 is read-modify-written rather than composed from scratch: poll it,
// flip the two bits, send it back. That way no other setting is disturbed and
// there is no need to get all forty bytes right.
#define UBX_NAVX5_AOPCFG_OFFSET 27      // aopCfg byte, u-blox 8 payload
#define UBX_NAVX5_MASK1_AOP     0x4000  // mask1 bit 14: apply the AOP settings

static void printAopStatus(HardwareSerial &gps, const char *when) {
  uint8_t buf[64];
  ubxSend(gps, 0x01, 0x60, nullptr, 0);                 // poll NAV-AOPSTATUS
  const int n = ubxWait(gps, 0x01, 0x60, buf, sizeof(buf), 2000);
  if (n < 0) { Serial.printf("      %-12s NAV-AOPSTATUS: sin respuesta\n", when); return; }
  Serial.printf("      %-12s aopCfg=0x%02X  status=0x%02X  %s\n", when, buf[4], buf[5],
                buf[5] ? "<- CALCULANDO predicciones" : "(en reposo)");
}

static void testAOP() {
  Serial.println("AssistNow Autonomous: se puede activar, y hace algo?");

  gpsPowerUp();
  HardwareSerial gpsSer(1);
  gpsSer.setRxBufferSize(1024);
  gpsSer.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(600);
  while (gpsSer.available()) gpsSer.read();

  uint8_t cfg[64];
  Serial.println("  [1] leyendo CFG-NAVX5...");
  ubxSend(gpsSer, 0x06, 0x23, nullptr, 0);
  int n = ubxWait(gpsSer, 0x06, 0x23, cfg, sizeof(cfg), 3000);
  if (n < 0) { Serial.println("      sin respuesta."); gpsPowerDown(gpsSer); return; }
  Serial.printf("      %d bytes, aopCfg actual = 0x%02X (%s)\n",
                n, cfg[UBX_NAVX5_AOPCFG_OFFSET],
                (cfg[UBX_NAVX5_AOPCFG_OFFSET] & 0x01) ? "AOP YA ACTIVO" : "AOP apagado");

  Serial.println("  [2] activando useAOP y reenviando...");
  cfg[2] |= (UBX_NAVX5_MASK1_AOP & 0xFF);          // mask1, byte bajo
  cfg[3] |= (UBX_NAVX5_MASK1_AOP >> 8) & 0xFF;     // mask1, byte alto
  cfg[UBX_NAVX5_AOPCFG_OFFSET] |= 0x01;            // useAOP
  ubxSend(gpsSer, 0x06, 0x23, cfg, (uint16_t)n);
  uint8_t ack[16];
  Serial.println(ubxWait(gpsSer, 0x05, 0x01, ack, sizeof(ack), 3000) >= 0
                 ? "      ACK recibido." : "      sin ACK.");

  Serial.println("  [3] releyendo...");
  ubxSend(gpsSer, 0x06, 0x23, nullptr, 0);
  n = ubxWait(gpsSer, 0x06, 0x23, cfg, sizeof(cfg), 3000);
  if (n >= 0) {
    Serial.printf("      aopCfg = 0x%02X  %s\n", cfg[UBX_NAVX5_AOPCFG_OFFSET],
                  (cfg[UBX_NAVX5_AOPCFG_OFFSET] & 0x01) ? "<- ACTIVADO" : "<- NO ha quedado activo");
  }

  // Now the real question. AOP only builds predictions once the receiver is
  // tracking and has ephemeris, so watch it while it holds a fix.
  Serial.println("  [4] vigilando 4 min con el GPS en marcha (status != 0 = calculando):");
  TinyGPSPlus nmea;
  const uint32_t t0 = millis();
  uint32_t lastPoll = 0;
  bool everBusy = false, fixed = false;

  while (millis() - t0 < 240000) {
    while (gpsSer.available()) nmea.encode(gpsSer.read());
    if (!fixed && nmea.location.isValid() && nmea.satellites.value() > 0) {
      fixed = true;
      Serial.printf("      fix a los %lu s, %lu satelites\n",
                    (unsigned long)((millis() - t0) / 1000),
                    (unsigned long)nmea.satellites.value());
    }
    if (millis() - lastPoll >= 30000) {
      lastPoll = millis();
      char when[16];
      snprintf(when, sizeof(when), "%3lu s", (unsigned long)((millis() - t0) / 1000));
      uint8_t buf[64];
      ubxSend(gpsSer, 0x01, 0x60, nullptr, 0);
      const int m = ubxWait(gpsSer, 0x01, 0x60, buf, sizeof(buf), 1500);
      if (m < 0) Serial.printf("      %-12s NAV-AOPSTATUS: sin respuesta\n", when);
      else {
        if (buf[5]) everBusy = true;
        Serial.printf("      %-12s aopCfg=0x%02X  status=0x%02X  %s  (sat=%lu)\n",
                      when, buf[4], buf[5], buf[5] ? "<- CALCULANDO" : "(reposo)",
                      nmea.satellites.isValid() ? (unsigned long)nmea.satellites.value() : 0UL);
      }
    }
    delay(5);
  }

  Serial.println();
  Serial.println(everBusy
    ? "  -> el receptor SI genera predicciones. Merece la pena medir el arranque tras >2 h."
    : "  -> no se ha visto calcular. Puede necesitar mas tiempo encendido, o varias sesiones.");
  gpsPowerDown(gpsSer);
}

// ---- Satellite module over AT --------------------------------------------
// Queries only. Never AT+TX (nothing must radiate without a matched antenna)
// and never AT+SAVE_CFG (it writes the module's flash and has killed KIM1s).
static void testSatAT() {
  Serial.println("Modulo satelite: identificacion por AT (solo consultas).");
  pinMode(SAT_TX, OUTPUT);
  digitalWrite(SAT_TX, HIGH);          // UART idle before the module wakes
  digitalWrite(EN_SAT, HIGH);          // KIM: ON/OFF. Arribada: rail VARR
  delay(1500);                         // SAT_MODULE_BOOT_MS is 1000 in firmware

  HardwareSerial sat(2);
  sat.begin(9600, SERIAL_8N1, SAT_RX, SAT_TX);
  delay(100);

  const char *queries[] = { "AT+FW=?", "AT+ID=?", "AT+SN=?", "AT+AFMT=?",
                            "AT+PWR=?", "AT+KMAC=?" };
  bool any = false;
  for (const char *q : queries) {
    while (sat.available()) sat.read();
    sat.print(q); sat.print("\r");
    Serial.printf("  %-11s -> ", q);
    String out;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
      while (sat.available()) {
        const char c = sat.read();
        if (c == '\r') continue;
        if (c == '\n') { out += " | "; continue; }
        out += (c >= 32 && c < 127) ? c : '?';
      }
      delay(5);
    }
    out.trim();
    if (out.length()) { any = true; Serial.println(out); }
    else Serial.println("(silencio)");
  }

  sat.end();
  pinMode(SAT_TX, INPUT);
  digitalWrite(EN_SAT, LOW);

  if (!any) Serial.println("  nada ha contestado. Mirar shield, VKIM y el GPIO13.");
  else Serial.println("  -> +FW con 'KIM' = KIM1; cualquier otro que conteste = Arribada.");
}

// ---- Deep sleep, so the current can actually be metered -------------------
// The console cannot measure current: there is no instrument on this end. What
// it can do is leave the board in the state whose consumption matters, with
// every rail cut, and come back on its own.
//
// ⚠️ Measuring over USB is meaningless: the USB-serial chip and the module's
// regulator dominate. Meter in series with the BATTERY, USB unplugged.
static void testDeepSleep() {
  const uint32_t seconds = 60;
  Serial.printf("Deep sleep de %lu s.\n", (unsigned long)seconds);
  Serial.println("  Para que el numero valga: multimetro en serie con la BATERIA");
  Serial.println("  y el USB DESENCHUFADO. Por USB se mide el CH9102, no la boya.");
  Serial.println("  Ojo: con un shield KIM montado, el modulo sigue alimentado por");
  Serial.println("  VKIM (5 V permanentes) aunque su ON/OFF este en bajo.");
  Serial.flush();

  // Everything down, in the order the mission firmware uses.
  SD.end();
  SPI.end();
  pinMode(23, INPUT); pinMode(18, INPUT); pinMode(SD_CS, INPUT); pinMode(19, INPUT);
  pinMode(SAT_TX, OUTPUT); digitalWrite(SAT_TX, LOW);
  pinMode(GPS_TX, OUTPUT); digitalWrite(GPS_TX, LOW);
  digitalWrite(EN_SD,  LOW);
  digitalWrite(EN_GPS, LOW);
  digitalWrite(EN_SAT, LOW);
  digitalWrite(LED_R, LOW); digitalWrite(LED_Y, LOW); digitalWrite(LED_G, LOW);
  delay(50);

  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);   // the RTC alarm line, as in flight
  Serial.println("  durmiendo...");
  Serial.flush();
  esp_deep_sleep_start();
}

// ---------------------------------------------------------------- menu

static void menu() {
  Serial.println();
  Serial.println("=== PUESTA EN MARCHA V2 ===");
  Serial.println("  l  LEDs             b  botones");
  Serial.println("  i  escaneo I2C      g  medidor de bateria");
  Serial.println("  r  RTC              a  alarma RTC -> GPIO34");
  Serial.println("  w  load switches    d  tarjeta SD");
  Serial.println("  p  GPS              u  configuracion UBX del GPS");
  Serial.println("  t  corte del rail del transmisor");
  Serial.println("  f  fix del GPS + arranque en caliente    c  poner el RTC en hora");
  Serial.println("  k  modulo satelite por AT              z  deep sleep 60 s");
  Serial.println("  j  velocidad del fix: 3 arranques en frio");
  Serial.println("  o  AssistNow Autonomous (prediccion de efemerides)");
  Serial.println("  x  el 0x0B fantasma   ?  este menu");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_R, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_G, OUTPUT);
  digitalWrite(LED_R, LOW); digitalWrite(LED_Y, LOW); digitalWrite(LED_G, LOW);

  pinMode(PB_1, INPUT_PULLUP);
  pinMode(PB_2, INPUT_PULLUP);
  pinMode(PB_3, INPUT);            // GPIO39 is input-only, R18 pulls it up

  pinMode(EN_SAT, OUTPUT); digitalWrite(EN_SAT, LOW);
  pinMode(EN_SD,  OUTPUT); digitalWrite(EN_SD,  LOW);
  pinMode(EN_GPS, OUTPUT); digitalWrite(EN_GPS, LOW);

  pinMode(RTC_INT, INPUT);

  Wire.begin(21, 22);

  Serial.println("\nPlaca V2 arrancada. Todos los rails conmutados apagados.");
  menu();
}

void loop() {
  if (!Serial.available()) { delay(50); return; }
  const int c = Serial.read();
  while (Serial.available()) Serial.read();      // drop the rest of the line

  switch (c) {
    case 'l': testLeds();     break;
    case 'b': testButtons();  break;
    case 'i': testI2C();      break;
    case 'g': testGauge();    break;
    case 'r': testRTC();      break;
    case 'a': testAlarm();    break;
    case 'w': testSwitches(); break;
    case 'd': testSD();       break;
    case 'p': testGPS();      break;
    case 't': testSatRail();  break;
    case 'o': testAOP();     break;
    case 'j': testGpsSpeed(); break;
    case 'k': testSatAT();   break;
    case 'z': testDeepSleep(); break;
    case 'f': testFix();     break;
    case 'c': setRTC();      break;
    case 'u': testUBX();     break;
    case 'x': testGhost();   break;
    case '?': menu();         break;
    default: return;
  }
  Serial.println("--- listo");
}
