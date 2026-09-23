// Read-only probe of the signals that can tell a V1 board from a V2.
//
// Safe on both boards: nothing is driven except GPIO13, which on a V1 closes
// the GPS_KIM relay and on a V2 powers the satellite module - both harmless.
// GPIO27 (PB_3 on a V1, the GPS load switch on a V2) and GPIO39 are only read.
// No SD access, nothing written anywhere. Prints one report and repeats it
// every time a key is sent.

#include <Wire.h>

#define PIN_SAT_OR_RELAY 13
#define PIN_GPS_RX        4   // GPS TXD -> ESP32
#define PIN_GPS_TX        2
#define PIN_27           27
#define PIN_39           39
#define PIN_36           36
#define PIN_34           34

static bool i2cAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Reads a pin many times; a pin with a real pull reads the same every time,
// a floating one wanders.
static void sampleDigital(const char *label, int pin) {
  int ones = 0;
  for (int i = 0; i < 200; i++) { ones += digitalRead(pin); delayMicroseconds(500); }
  Serial.printf("  %-34s %3d/200 high\n", label, ones);
}

static void sampleAnalog(const char *label, int pin) {
  uint32_t mn = 5000, mx = 0, sum = 0;
  for (int i = 0; i < 64; i++) {
    const uint32_t mv = analogReadMilliVolts(pin);
    mn = min(mn, mv); mx = max(mx, mv); sum += mv;
    delay(2);
  }
  Serial.printf("  %-34s mean %4u mV  min %4u  max %4u\n", label, sum / 64, mn, mx);
}

static void report() {
  Serial.println("\n=== BOARD PROBE ===");

  Serial.println("I2C:");
  Serial.printf("  0x36 MAX17048 (V2, needs battery)  %s\n", i2cAck(0x36) ? "ACK" : "--");
  Serial.printf("  0x68 DS3231                        %s\n", i2cAck(0x68) ? "ACK" : "--");

  Serial.println("Inputs, nothing driven:");
  pinMode(PIN_39, INPUT);            sampleDigital("GPIO39 INPUT", PIN_39);
  sampleAnalog("GPIO39 analog", PIN_39);
  sampleAnalog("GPIO36 analog", PIN_36);
  pinMode(PIN_34, INPUT);            sampleDigital("GPIO34 INPUT", PIN_34);
  pinMode(PIN_27, INPUT);            sampleDigital("GPIO27 INPUT (no pull)", PIN_27);
  pinMode(PIN_27, INPUT_PULLDOWN);   sampleDigital("GPIO27 INPUT_PULLDOWN", PIN_27);
  pinMode(PIN_27, INPUT_PULLUP);     sampleDigital("GPIO27 INPUT_PULLUP", PIN_27);
  sampleAnalog("GPIO27 analog, pull-up on", PIN_27);
  pinMode(PIN_27, INPUT);

  Serial.println("GPS UART after GPIO13 high (V1: relay feeds the GPS):");
  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  while (Serial2.available()) Serial2.read();
  digitalWrite(PIN_SAT_OR_RELAY, HIGH);
  const uint32_t t0 = millis();
  uint32_t bytes = 0, dollars = 0, firstByteMs = 0;
  char sample[81] = {0}; int n = 0;
  while (millis() - t0 < 3000) {
    while (Serial2.available()) {
      const char c = Serial2.read();
      if (bytes == 0) firstByteMs = millis() - t0;
      bytes++;
      if (c == '$') dollars++;
      if (n < 80 && c >= 32 && c < 127) sample[n++] = c;
    }
  }
  digitalWrite(PIN_SAT_OR_RELAY, LOW);
  Serial2.end();
  pinMode(PIN_GPS_TX, INPUT);
  Serial.printf("  %u bytes, %u '$', first byte at %u ms\n", bytes, dollars, firstByteMs);
  Serial.printf("  sample: %s\n", sample);
  Serial.println("--- done (send any key to repeat)");
}

void setup() {
  pinMode(PIN_SAT_OR_RELAY, OUTPUT);
  digitalWrite(PIN_SAT_OR_RELAY, LOW);
  Serial.begin(115200);
  Wire.begin();
  delay(1500);  // let the relay/rails settle after reset
  report();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    report();
  }
}
