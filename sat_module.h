#pragma once
#include <Arduino.h>

// Runtime abstraction over the two satellite transmitters the buoy can carry.
//
// The KIM1 shield and the Arribada Argos SMD wing plug into the same socket:
// same UART (GPIO16/17), same enable GPIO, never both at once. One firmware
// therefore has to work with either, deciding which one at boot instead of at
// compile time. Everything above this layer (satellite_tx.cpp, the sketch) is
// written against these functions and never touches KIM or ARRIBADA directly.
//
// The two AT dialects are close but not identical. The differences that matter:
//   - KIM1 has AT+PWR and AT+AFMT; the Arribada firmware has neither (its RF
//     level lives inside the AT+RCONF blob), so those calls become no-ops.
//   - KIM1 has no AT+PING; the Arribada firmware does. That asymmetry is what
//     makes detection reliable.
//   - Payload length rules differ; see satModuleSendData().

enum SatModuleType {
  SAT_UNKNOWN = 0,   // not probed yet, or no module answered
  SAT_KIM1    = 1,
  SAT_ARRIBADA = 2
};

// Probes the socket and decides which module is fitted. Also reads the module
// ID and cross-checks it against the ID->type list on the SD card, logging any
// disagreement. Safe to call more than once; the result is cached.
// Returns the detected type (SAT_UNKNOWN if nothing answered).
SatModuleType satModuleDetect();

// Cached result of satModuleDetect(), without re-probing the hardware.
SatModuleType satModuleType();

// "KIM1", "ARRIBADA" or "UNKNOWN" - for logs and messages.
const char *satModuleName();

// Link check against whichever module was detected.
bool satModuleCheck();

// Module ID as reported over AT, stripped of its "+ID:"/"+ID=" wrapper.
// The pointer stays valid until the next call.
const char *satModuleGetID();

// Serial number as reported over AT (full response line, module specific).
const char *satModuleGetSN();

// TX power in mW as a decimal string ("1000", "100", ...).
// KIM1: AT+PWR. Arribada: not supported, logged and reported as success so the
// caller's flow is unchanged.
bool satModuleSetPower(const char *powerMilliWatt);

// Argos message format. KIM1: AT+AFMT. Arribada: not applicable, no-op.
bool satModuleSetFormat(const char *format);

// Transmits one hex-string payload, padding it to the Argos word boundary
// first (see the .cpp for why). Returns true when the module accepted it.
bool satModuleSendData(const char *hexPayload);

// Releases the UART and tri-states its pins before the module is powered down.
// The next call that talks to the module opens it again.
void satModuleEnd();

// satModuleEnd() plus the transmit line held low. Use it before the module's
// supply or ON/OFF drops: an idle-high TX line feeds the module through its
// receive pin, and on a V2 - where VKIM stays up and GPIO13 is only ON/OFF -
// it keeps the KIM awake in Standby at 3.5 mA (measured 40/40 alive, 22 Sep).
void satModuleReleaseLines();

// The other half, and it has to happen the moment the module is switched back
// on: reopen the port so the transmit line sits at its idle high while the
// module boots. A KIM that boots with its RX held low reads it as garbage and
// rejects the next command - AT+TX answered +ERROR=6, 2 out of 2 on the V2
// bench, against +OK 2 out of 2 with the port reopened at power-on (23 Sep).
void satModuleRestoreLines();

// Name of the SD file mapping module IDs to module types, one "ID;TYPE" per
// line (TYPE being KIM1 or ARR), e.g. "26423D4;KIM1" / "294847;ARR".
extern const char *SD_satmodule_filename;

// Radio configuration the Arribada is expected to be running: the Argos uplink
// band (401.65 MHz +/- 30 kHz, where the KIM1 also sits), maximum power and the
// same modulation. Compared as a substring of the module's AT+RCONF reply.
//
// The buoy only ever *checks* this and writes a log line if it differs. It does
// not correct it, by design: AT+RCONF erases and reprograms the flash page that
// also holds the device ID, address and secret key. Changing it is a deliberate
// manual job (in practice, reflashing the module), not something firmware
// should be doing on every wake.
#define SAT_ARRIBADA_EXPECTED_RCONF "401620000,401680000,27,LDA2"

// ---- Recovery --------------------------------------------------------------

// Drops the cached detection result so the next satModuleDetect() probes the
// hardware again. Used after a power cycle, when the module may have changed
// its mind about being alive.
void satModuleForget();

// Cuts the module's supply rail, brings it back and re-probes. The only reset
// available: the ESP32 can switch the relay and nothing finer. Returns true if
// the module answers afterwards. See SAT_MODULE_MAX_ATTEMPTS in conf.h for why
// this exists.
bool satModulePowerCycle();
