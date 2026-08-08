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
#define ADC_RESOLUTION 8  // ADC resolution in bits

void adcSetup() {
  analogReadResolution(ADC_RESOLUTION);
}

void adcAcquireData(char *ADCreadHex) {

  float R1_ADC = 10; // Resistance R1 value in MΩ
  float R2_ADC = 10; // Resistance R2 value in MΩ
  float Vref_ADC = 3.3; // Tensión de referencia del ADC (3.3V)
  float a_calADC = 1; //0.9637; // calibration as y=ax+b
  float b_calADC = 0; //0.500;
  float Vout_ADC;
  int ADCread;
  int sumaADCread = 0;

  for (int i = 0; i < 100; ++i) {
    int ADCread = analogRead(ADC_PIN);  // Realiza la lectura analógica
    sumaADCread += ADCread;  // Suma la lectura actual a la suma total
    delay(10);  // Espera antes de la próxima lectura
  }

  ADCread = sumaADCread/100; // Realizar la lectura analógica

  // Calcular el voltaje antes del divisor de tensión basado en la lectura del ADC y la referencia de voltaje
  Vout_ADC = (ADCread*Vref_ADC) /(pow(2, ADC_RESOLUTION) - 1);

  // Calcular el valor en voltaje basado en la lectura del ADC y el divisor de tensión + calibración
  Vin_ADC = Vout_ADC * ((R1_ADC + R2_ADC) / R2_ADC)*a_calADC+b_calADC;

  // Convertir la lectura de 8 bits a hexadecimal (2 caracteres) y mostrarlo
  sprintf(ADCreadHex, "%02X", ADCread); // Convertir a hexadecimal
  delay(100);

  // Mostrar la lectura del ADC en 8 bits
  SerialPrintDebug("ADC read (8 bits): ");
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

  // DS3231 die temperature (updated internally every 64 s, +-3 C, not ambient-calibrated) --
  // logged each cycle to check the overheating-in-the-sun hypothesis.
  float rtcTempC = rtcExt.getTemperature();
  SerialPrintDebug("RTC temp: ");
  SerialPrintDebugln(String(rtcTempC) + " C");
  writeLogFile("RTC temp: " + String(rtcTempC) + " C");
}
