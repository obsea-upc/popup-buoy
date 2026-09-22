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

// Whether a battery pack is fitted at all, as opposed to being flat. False means
// the buoy is running off the USB cable with no pack in it, which must not be
// mistaken for a critical battery - see BAT_ABSENT_V in conf.h. Only meaningful
// after adcAcquireData() has run this cycle.
bool batteryPresent();

// Battery voltage and last ADC hex reading (defined in adc.cpp).
extern float Vin_ADC;
extern char ADCreadHex[3];

// Per-board trim of the battery reading, y = a*x + b, in volts. Filled from
// conf.txt (ADC_CAL_A_x1000, ADC_CAL_B_mV); 1.0 / 0.0 leave the reading alone.
// The chip's own factory calibration is already applied inside adcAcquireData(),
// so this only takes out what is left: the divider's resistor tolerance. Measure
// the pack with a multimeter, compare with the "Vin (up)" line, and trim.
extern float adcCalA;
extern float adcCalB;
