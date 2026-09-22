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

// Each load switch on its own, so the rail can be metered while it is up.
static void testSwitches() {
  struct { int pin; const char *name; const char *rail; } sw[3] = {
    { EN_SD,  "SD  (GPIO14)", "3.3_REG_SD"  },
    { EN_GPS, "GPS (GPIO27)", "3.3_REG_GPS" },
    { EN_SAT, "SAT (GPIO13)", "3.3_REG_ARR / ON_OFF del KIM" },
  };
  for (int i = 0; i < 3; i++) {
    Serial.printf("%s ON  -> mide %s, 5 s\n", sw[i].name, sw[i].rail);
    digitalWrite(sw[i].pin, HIGH);
    delay(5000);
    digitalWrite(sw[i].pin, LOW);
    Serial.printf("%s OFF -> debe caer a 0 V\n", sw[i].name);
    delay(1500);
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

static void testGPS() {
  Serial.println("GPS: alimentando el rail (GPIO27) y escuchando 10 s a 9600...");
  digitalWrite(EN_GPS, HIGH);
  delay(200);
  HardwareSerial gps(1);
  gps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  const uint32_t t0 = millis();
  int n = 0;
  while (millis() - t0 < 10000) {
    while (gps.available()) { Serial.write(gps.read()); n++; }
    delay(5);
  }
  gps.end();
  Serial.printf("\n  %d bytes. Sin fix en interior es normal; lo que importa es que hable.\n", n);
  Serial.println("  -> un NEO-M8N genuino emite $GNxxx y responde a UBX (a probar luego).");
  digitalWrite(EN_GPS, LOW);
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

// ---------------------------------------------------------------- menu

static void menu() {
  Serial.println();
  Serial.println("=== PUESTA EN MARCHA V2 ===");
  Serial.println("  l  LEDs             b  botones");
  Serial.println("  i  escaneo I2C      g  medidor de bateria");
  Serial.println("  r  RTC              a  alarma RTC -> GPIO34");
  Serial.println("  w  load switches    d  tarjeta SD");
  Serial.println("  p  GPS              t  corte del rail del transmisor");
  Serial.println("  ?  este menu");
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
    case '?': menu();         break;
    default: return;
  }
  Serial.println("--- listo");
}
