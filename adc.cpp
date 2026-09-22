#include "adc.h"
#include "conf.h"      // for ADC_PIN
#include "logging.h"
#include <RTClib.h>

// Battery voltage + ADC hex reading, owned by this module (declared extern in adc.h).
float Vin_ADC;      // read by the state machine's battery checks
char ADCreadHex[3]; // 2 hex chars, put into the Kineis message by maskGPS

// RTC owned by the main sketch (popup-buoy.ino); read here for the once-per-cycle die temperature log.
extern RTC_DS3231 rtcExt;

// --- Current (analog) hardware only. On the new board this becomes an I2C read. ---
// Resolution of the value that travels in the Kineis message. The reading itself is
// taken at 12 bits and in millivolts; this is only the scale of the transmitted byte,
// kept at 8 bits so the payload and every decoder written for it stay unchanged:
//   volts = byte * 3.3 / 255 * 2
#define ADC_MSG_BITS 8
#define ADC_MSG_VOLTS_PER_STEP (3.3f / 255.0f * 2.0f)   // 25.88 mV

// Per-board trim of the battery reading, y = a*x + b, filled from conf.txt
// (ADC_CAL_A_x1000 and ADC_CAL_B_mV). 1.0 and 0.0 mean "no trim".
float adcCalA = 1.0f;
float adcCalB = 0.0f;

void adcSetup() {
  // 12 bits and the 11 dB attenuation the divider needs (0-3.3 V at the pin).
  // analogReadMilliVolts() below returns millivolts whatever the resolution, but
  // reading at 12 bits keeps the averaging meaningful.
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
}

bool batteryPresent() {
  return Vin_ADC >= BAT_ABSENT_V;
}

void adcAcquireData(char *ADCreadHex) {

  float R1_ADC = 10; // Resistance R1 value in MΩ
  float R2_ADC = 10; // Resistance R2 value in MΩ
  float Vout_ADC;
  int ADCread;
  uint32_t sumaMilliVolts = 0;

  // analogReadMilliVolts() applies the chip's own factory calibration (the Vref
  // burned in its eFuse) and the attenuation curve. The old code instead assumed
  // the pin reaches full scale at exactly 3.3 V, which no ESP32 does: the real
  // reference lands anywhere between about 1000 and 1200 mV, so the same battery
  // read differently on every board. Measured 17 Sep 2026 with four buoys charged
  // to the same 4.2 V: 4.30 V on buoy 5 and 4.71 V on buoy 1 - a voltage the pack
  // cannot physically reach. Everything downstream (Bat_critlevel, BAT_ABSENT_V,
  // the battery byte in the message) was reading that error.
  for (int i = 0; i < 100; ++i) {
    sumaMilliVolts += analogReadMilliVolts(ADC_PIN);
    delay(10);  // Espera antes de la próxima lectura
  }

  Vout_ADC = (sumaMilliVolts / 100.0f) / 1000.0f;   // volts at the pin, calibrated

  // Battery voltage before the divider, plus the per-board trim from conf.txt.
  Vin_ADC = Vout_ADC * ((R1_ADC + R2_ADC) / R2_ADC) * adcCalA + adcCalB;

  // The message still carries one byte on the old scale, so a decoder written for
  // earlier tests keeps working - it now decodes a corrected voltage.
  ADCread = (int)roundf(Vin_ADC / ADC_MSG_VOLTS_PER_STEP);
  if (ADCread < 0) ADCread = 0;
  if (ADCread > 255) ADCread = 255;
  sprintf(ADCreadHex, "%02X", ADCread); // Convertir a hexadecimal
  delay(100);

  // Mostrar la lectura del ADC en 8 bits
  SerialPrintDebug("ADC byte (8 bits): ");
  SerialPrintDebugln(ADCread);
  // Mostrar el valor de voltaje calculado después del divisor de tensión
  SerialPrintDebug("Vout (in the ADC): ");
  char buffer[10];
  dtostrf(Vout_ADC, 6, 5, buffer);
  SerialPrintDebug(buffer);
  SerialPrintDebugln(" V");
  // Mostrar el voltaje antes del divisor de tensión
  SerialPrintDebug("Vin (up): ");
  dtostrf(Vin_ADC, 6, 5, buffer);
  SerialPrintDebug(buffer);
  SerialPrintDebugln(" V");
  writeLogFile("Vin (up): " + String(buffer) + " V");

  // Say it out loud rather than leaving it to be inferred from the voltage. A
  // buoy on the bench reads about 2.1 V and would otherwise look like a critical
  // battery in the log, which is exactly the confusion this is here to end.
  if (!batteryPresent()) {
    writeLogFile("No battery pack detected (below " + String(BAT_ABSENT_V)
                 + " V): running on external power. Battery alerts suppressed.");
  }

  // DS3231 die temperature (updated internally every 64 s, +-3 C, not ambient-calibrated) --
  // logged each cycle to check the overheating-in-the-sun hypothesis.
  float rtcTempC = rtcExt.getTemperature();
  SerialPrintDebug("RTC temp: ");
  SerialPrintDebugln(String(rtcTempC) + " C");
  writeLogFile("RTC temp: " + String(rtcTempC) + " C");
}
