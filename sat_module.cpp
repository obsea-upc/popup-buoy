#include "sat_module.h"
#include "conf.h"
#include "logging.h"
#include "KIM.h"
#include "ARRIBADA.h"
#include <SD.h>
#include <string.h>
#include <ctype.h>

// Both drivers are constructed in the sketch on the same HardwareSerial, which
// detection drives directly before handing over to one of them.
extern KIM KIM;
extern ARRIBADA Arribada;
extern HardwareSerial kimSerial;

const char *SD_satmodule_filename = "/sat_transm.csv";

static SatModuleType detectedType = SAT_UNKNOWN;
static char moduleID[32] = "";

// 23 bytes, the largest payload the Kineis standard format carries.
#define SAT_MAX_HEX_PAYLOAD 46

// --------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------

// Copies the bare ID out of an AT reply into moduleID.
// Measured on a KIM1 FW 2.1: "+ID=276f235" (lowercase hex). The Arribada
// answers "+ID=294847" (decimal), and older KIM headers document "+ID:28,...".
// Take whatever follows the last ':', '=' or ',' and strip the whitespace, so
// all three shapes reduce to the bare ID the SD list is keyed on.
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

// Pads a hex payload up to the next length the Kineis standard format accepts.
//
// KIM1 Integration Manual v2.3 section 4.2.2 lists the only user-data sizes the
// standard format (AT+AFMT=1,16,32) carries: 3, 7, 11, 15, 19 and 23 bytes,
// i.e. 6, 14, 22, 30, 38 and 46 hex characters - the "6+8n" rule. Anything else
// "will be zero-padded by the KIM1 until it reaches the next possible data
// length value", and anything longer is truncated.
//
// The ground segment decodes by received length (position-sized payload -> parse
// lat/long/epoch, longer -> keep as raw data), so the Arribada has to put the
// same sizes on the air as the KIM1 does. That means padding to this table and
// not to a plain multiple of 8: a 26-char position message becomes 30 (15
// bytes), not 32, and a 46-char data line is already the maximum and stays put.
//
// Returns the padded length, or 0 if the payload is longer than the format
// allows (better to log and skip than to let the module silently truncate).
static const size_t SAT_VALID_HEX_LENGTHS[] = {6, 14, 22, 30, 38, 46};

static size_t padToArgosLength(const char *in, char *out, size_t outSize) {
  if (in == nullptr || out == nullptr) return 0;

  size_t len = strlen(in);
  size_t padded = 0;

  for (size_t i = 0; i < sizeof(SAT_VALID_HEX_LENGTHS) / sizeof(SAT_VALID_HEX_LENGTHS[0]); i++) {
    if (SAT_VALID_HEX_LENGTHS[i] >= len) {
      padded = SAT_VALID_HEX_LENGTHS[i];
      break;
    }
  }

  if (padded == 0 || padded + 1 > outSize) return 0;

  memcpy(out, in, len);
  for (size_t i = len; i < padded; i++) out[i] = '0';
  out[padded] = '\0';

  return padded;
}

// --------------------------------------------------------------------------
// Raw AT probing, used only while working out which module is fitted
// --------------------------------------------------------------------------

// Sends one AT command and collects whatever comes back for windowMs into reply.
static void probeAT(const char *cmd, char *reply, size_t replySize, uint32_t windowMs) {
  reply[0] = '\0';
  if (cmd == nullptr) return;

  while (kimSerial.available()) kimSerial.read();   // drop stale bytes
  kimSerial.print(cmd);
  kimSerial.print("\r");

  size_t n = 0;
  uint32_t start = millis();
  while (millis() - start < windowMs && n < replySize - 1) {
    while (kimSerial.available() && n < replySize - 1) {
      char c = (char)kimSerial.read();
      if (c == '\r' || c == '\n') {
        if (n > 0) { reply[n] = '\0'; return; }   // first complete line is enough
        continue;
      }
      reply[n++] = c;
    }
    delay(5);
  }
  reply[n] = '\0';
}

// --------------------------------------------------------------------------
// Detection
// --------------------------------------------------------------------------

SatModuleType satModuleDetect() {
  if (detectedType != SAT_UNKNOWN) return detectedType;

  SerialPrintDebugln("Satellite module detection ---->");

  // Ask the module to name itself rather than inferring from which commands it
  // knows. Measured on a KIM1 (FW 2.1): "AT+FW=?" -> "+FW=KIM1_V2.1", and the
  // Arribada answers its AT+FW with a bare git commit id. Two more obvious
  // discriminators were tried first and both turned out to be wrong:
  //   - AT+PING: the KIM1 answers "+OK" too (integration manual v2.3 section
  //     3.3.3.b), so it does not single out the Arribada.
  //   - the AT+ID reply format: KIM1 FW 2.1 answers "+ID=276f235", the same
  //     "+ID=" shape as the Arribada, not the "+ID:28,..." older headers show.
  char reply[64];

  kimSerial.begin(KIMBaud, SERIAL_8N1, RX_KIM, TX_KIM);
  delay(50);

  probeAT("AT+FW=?", reply, sizeof(reply), 1500);          // KIM1 syntax
  if (strstr(reply, "KIM") != nullptr) {
    detectedType = SAT_KIM1;
  } else {
    probeAT("AT+FW?", reply, sizeof(reply), 1500);         // Arribada syntax
    if (strncmp(reply, "+FW=", 4) == 0 && strstr(reply, "KIM") == nullptr) {
      detectedType = SAT_ARRIBADA;
    } else {
      // Last resort: AT+AFMT exists only on the KIM1; the Arribada answers
      // +ERROR=1203 (unknown AT command).
      probeAT("AT+AFMT=?", reply, sizeof(reply), 1500);
      if (strncmp(reply, "+AFMT=", 6) == 0) detectedType = SAT_KIM1;
    }
  }

  kimSerial.end();

  // Hand the port over to the driver that won, and read the ID through it.
  if (detectedType == SAT_KIM1) {
    if (KIM.check()) {
      extractID(KIM.get_ID());
    } else {
      detectedType = SAT_UNKNOWN;
    }
  } else if (detectedType == SAT_ARRIBADA) {
    Arribada.begin(KIMBaud, RX_KIM, TX_KIM);
    if (Arribada.check()) {
      extractID(Arribada.get_ID());
    } else {
      Arribada.end();
      detectedType = SAT_UNKNOWN;
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
      // This firmware pads nothing: a payload whose length is not one the format
      // accepts comes back as +ERROR=1100. Do the padding the KIM1 would have
      // done, so both modules put the same sizes on the air and the ground
      // segment can keep decoding by length, and so the SD files stay unedited.
      char padded[SAT_MAX_HEX_PAYLOAD + 1];
      size_t paddedLen = padToArgosLength(hexPayload, padded, sizeof(padded));
      if (paddedLen == 0) {
        writeLogFile("SAT MSG_ERR: payload too long (" + String(len) + " hex chars, max " + String(SAT_MAX_HEX_PAYLOAD) + ")");
        return false;
      }
      if (Arribada.send_data(padded, paddedLen) == OK_ARRIBADA) return true;
      // Worth spelling out: if this turns out to be +ERROR=1100 the Arribada
      // wants 4-byte-aligned payloads instead of the Kineis table, and the two
      // modules cannot produce identical lengths. Needs a real Arribada to tell.
      writeLogFile("ARRIBADA MSG_ERR sending " + String(paddedLen) + " hex chars");
      return false;
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
