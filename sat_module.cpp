#include "sat_module.h"
#include "conf.h"
#include "logging.h"
#include "KIM.h"
#include "ARRIBADA.h"
#include <SD.h>
#include <string.h>
#include <ctype.h>

// Both drivers are constructed in the sketch on the same HardwareSerial.
extern KIM KIM;
extern ARRIBADA Arribada;

const char *SD_satmodule_filename = "/sat_transm.csv";

static SatModuleType detectedType = SAT_UNKNOWN;
static char moduleID[32] = "";

// Longest payload the Argos LDA2/VLD services carry: 24 bytes = 48 hex chars.
#define SAT_MAX_HEX_PAYLOAD 48

// --------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------

// Copies the bare ID out of an AT reply into moduleID.
// KIM1 answers "+ID:28,7AC8998" (hex ID after the comma); the Arribada firmware
// answers "+ID=294847" (decimal). Take whatever follows the last ':', '=' or
// ',' and strip the whitespace.
static void extractID(const char *response) {
  moduleID[0] = '\0';
  if (response == nullptr) return;

  const char *start = response;
  for (const char *p = response; *p != '\0'; p++) {
    if (*p == ':' || *p == '=' || *p == ',') start = p + 1;
  }

  while (*start != '\0' && isspace((unsigned char)*start)) start++;

  size_t n = 0;
  while (start[n] != '\0' && n < sizeof(moduleID) - 1 &&
         !isspace((unsigned char)start[n])) {
    moduleID[n] = start[n];
    n++;
  }
  moduleID[n] = '\0';
}

// Looks the ID up in the SD list and returns SAT_KIM1 / SAT_ARRIBADA, or
// SAT_UNKNOWN when the file or the entry is missing. Never fatal: the file is
// documentation that confirms the probe, not a prerequisite for transmitting.
static SatModuleType lookupIDinFile(const char *id) {
  if (id == nullptr || id[0] == '\0') return SAT_UNKNOWN;

  File f = SD.open(SD_satmodule_filename, FILE_READ);
  if (!f) {
    SerialPrintDebugln(String(SD_satmodule_filename) + " not found, skipping ID cross-check");
    return SAT_UNKNOWN;
  }

  SatModuleType found = SAT_UNKNOWN;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int sep = line.indexOf(';');
    if (sep < 0) continue;

    String fileID = line.substring(0, sep);
    String fileType = line.substring(sep + 1);
    fileID.trim();
    fileType.trim();

    if (fileID.equalsIgnoreCase(id)) {
      if (fileType.equalsIgnoreCase("KIM1")) {
        found = SAT_KIM1;
      } else if (fileType.equalsIgnoreCase("ARR") || fileType.equalsIgnoreCase("ARRIBADA")) {
        found = SAT_ARRIBADA;
      }
      break;
    }
  }
  f.close();
  return found;
}

// Pads a hex payload up to a whole number of 32-bit Argos words.
//
// Argos user data is carried in 32-bit words, so the hex string has to be a
// multiple of 8 characters. The KIM1 firmware accepts 6+8n and zero-fills the
// rest by itself; the Arribada firmware refuses anything else outright with
// +ERROR=1100 (invalid user data length). Padding here means the seabed
// dataFile lines (46 chars -> 48, i.e. exactly the 24-byte LDA2 maximum) and
// the GPS message (26 -> 32) feed either module without editing the files.
//
// Returns the padded length, or 0 if the payload does not fit.
static size_t padToArgosWord(const char *in, char *out, size_t outSize) {
  if (in == nullptr || out == nullptr) return 0;

  size_t len = strlen(in);
  size_t padded = ((len + 7) / 8) * 8;

  if (padded > SAT_MAX_HEX_PAYLOAD || padded + 1 > outSize) return 0;

  memcpy(out, in, len);
  for (size_t i = len; i < padded; i++) out[i] = '0';
  out[padded] = '\0';

  return padded;
}

// --------------------------------------------------------------------------
// Detection
// --------------------------------------------------------------------------

SatModuleType satModuleDetect() {
  if (detectedType != SAT_UNKNOWN) return detectedType;

  SerialPrintDebugln("Satellite module detection ---->");

  // Probe the Arribada first. AT+PING exists only in its firmware, so a reply
  // is conclusive. The reverse order would not work: KIM::check() merely looks
  // for a leading '+', and the Arribada's "+ID=..." would pass that test.
  Arribada.begin(KIMBaud, RX_KIM, TX_KIM);
  if (Arribada.check()) {
    detectedType = SAT_ARRIBADA;
    extractID(Arribada.get_ID());
  } else {
    Arribada.end();
    if (KIM.check()) {
      detectedType = SAT_KIM1;
      extractID(KIM.get_ID());
    }
  }

  if (detectedType == SAT_UNKNOWN) {
    writeLogFile("Satellite module NOT DETECTED (no KIM1, no Arribada)");
    SerialPrintDebugln("Satellite module detection ----> NONE");
    return detectedType;
  }

  // Cross-check against the SD list. A disagreement means the wrong shield is
  // fitted or the list is stale: log it loudly but keep the probed type, which
  // is the one that actually answered.
  SatModuleType fromFile = lookupIDinFile(moduleID);
  if (fromFile == SAT_UNKNOWN) {
    writeLogFile("SAT module " + String(satModuleName()) + " ID " + String(moduleID) +
                 " not listed in " + String(SD_satmodule_filename));
  } else if (fromFile != detectedType) {
    writeLogFile("SAT module MISMATCH: hardware says " + String(satModuleName()) +
                 " but " + String(SD_satmodule_filename) + " lists ID " + String(moduleID) +
                 " as the other type. Using the hardware.");
  } else {
    writeLogFile("SAT module " + String(satModuleName()) + " ID " + String(moduleID) + " (confirmed by SD list)");
  }

  SerialPrintDebugln("Satellite module detection ----> " + String(satModuleName()) + " ID " + String(moduleID));
  return detectedType;
}

SatModuleType satModuleType() {
  return detectedType;
}

const char *satModuleName() {
  switch (detectedType) {
    case SAT_KIM1:     return "KIM1";
    case SAT_ARRIBADA: return "ARRIBADA";
    default:           return "UNKNOWN";
  }
}

// --------------------------------------------------------------------------
// Uniform operations
// --------------------------------------------------------------------------

bool satModuleCheck() {
  switch (detectedType) {
    case SAT_KIM1:     return KIM.check();
    case SAT_ARRIBADA: return Arribada.check();
    default:           return satModuleDetect() != SAT_UNKNOWN;
  }
}

const char *satModuleGetID() {
  switch (detectedType) {
    case SAT_KIM1:     extractID(KIM.get_ID());      break;
    case SAT_ARRIBADA: extractID(Arribada.get_ID()); break;
    default: break;
  }
  return moduleID;
}

const char *satModuleGetSN() {
  switch (detectedType) {
    case SAT_KIM1:     return KIM.get_SN();
    case SAT_ARRIBADA: return Arribada.get_SN();
    default:           return "";
  }
}

bool satModuleSetPower(const char *powerMilliWatt) {
  if (powerMilliWatt == nullptr) return false;

  switch (detectedType) {
    case SAT_KIM1:
      return KIM.set_PWR((char *)powerMilliWatt, strlen(powerMilliWatt)) == OK_KIM;

    case SAT_ARRIBADA:
      // No AT+PWR in this firmware: the RF level is part of the 128-bit AT+RCONF
      // blob and is left at whatever the module was provisioned with. Report
      // success so the caller's per-state logic stays untouched.
      writeLogFile("SAT power " + String(powerMilliWatt) + " mW requested, but ARRIBADA has no AT+PWR (set via RCONF at provisioning) - ignored");
      return true;

    default:
      return false;
  }
}

bool satModuleSetFormat(const char *format) {
  if (format == nullptr) return false;

  switch (detectedType) {
    case SAT_KIM1:
      return KIM.set_AFMT((char *)format, strlen(format)) == OK_KIM;

    case SAT_ARRIBADA:
      // No AT+AFMT: the Arribada firmware always frames the payload itself.
      return true;

    default:
      return false;
  }
}

bool satModuleSendData(const char *hexPayload) {
  if (hexPayload == nullptr || hexPayload[0] == '\0') return false;

  size_t len = strlen(hexPayload);

  switch (detectedType) {
    case SAT_KIM1:
      // Hand the payload over untouched. The KIM1 has been flying with these
      // exact lengths (26 hex chars for a position, 46 for a data line) and
      // does its own zero-filling, so padding here would change the bits that
      // actually go on the air and the ground segment decodes.
      if (len > SAT_MAX_HEX_PAYLOAD) {
        writeLogFile("SAT MSG_ERR: payload too long (" + String(len) + " hex chars)");
        return false;
      }
      return KIM.send_data((char *)hexPayload, len) == OK_KIM;

    case SAT_ARRIBADA: {
      // This firmware does no padding of its own: a payload that is not a whole
      // number of 32-bit words comes back as +ERROR=1100. Pad it here so the
      // same SD files feed both modules unedited.
      char padded[SAT_MAX_HEX_PAYLOAD + 1];
      size_t paddedLen = padToArgosWord(hexPayload, padded, sizeof(padded));
      if (paddedLen == 0) {
        writeLogFile("SAT MSG_ERR: payload too long (" + String(len) + " hex chars)");
        return false;
      }
      return Arribada.send_data(padded, paddedLen) == OK_ARRIBADA;
    }

    default:
      return false;
  }
}

void satModuleEnd() {
  if (detectedType == SAT_ARRIBADA) {
    Arribada.end();
  }
  // The KIM driver releases its UART inside set_sleepMode(true), which the
  // existing power-down path already calls.
}
