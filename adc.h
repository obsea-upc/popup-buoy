#pragma once
#include <Arduino.h>

// Battery-voltage acquisition.
//
// CURRENT HARDWARE: rudimentary analog read of ADC_PIN through a resistor divider.
// NEW HARDWARE (in design): the ADC becomes a digital module over I2C. Only the
// bodies in adc.cpp change; this interface and the outputs stay the same:
//   - global Vin_ADC  <- measured battery voltage (read by the battery checks)
//   - ADCreadHex      <- raw ADC value as 2 hex chars, for the Kineis message

// One-time ADC configuration (called from setup on surface states).
void adcSetup();

// Sample the battery: fills the global Vin_ADC and writes 2 hex chars to ADCreadHex.
void adcAcquireData(char *ADCreadHex);
