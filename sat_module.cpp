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

// Largest payload each module takes: 23 bytes for the Kineis standard format
// on the KIM1, 24 bytes for LDA2/VLD on the Arribada.
#define SAT_MAX_HEX_KIM      46
#define SAT_MAX_HEX_ARRIBADA 48
#define SAT_MAX_HEX_PAYLOAD  SAT_MAX_HEX_ARRIBADA

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

// Zero-pads a hex payload for the Arribada, whose firmware pads nothing itself
// and answers +ERROR=1100 on any length it does not accept.
//
// The two modules do not agree on which lengths are legal, and there is no
// overlap to aim for:
//   - KIM1, Kineis standard format (integration manual v2.3 section 4.2.2):
//     3, 7, 11, 15, 19, 23 bytes = 6, 14, 22, 30, 38, 46 hex chars ("6+8n").
//     Short payloads are zero-padded by the module up to the next entry.
//   - Arribada: whole 4-byte words only, up to 24 bytes = multiples of 8 hex
//     chars, 48 maximum ("8n"). Measured: it rejects 30 and 46.
//
// 6+8n and 8n never coincide, so the two transmitters cannot put the same
// length on the air. The buoy therefore sends each module the nearest size it
// accepts and the ground segment learns both: a position message goes out as
// 30 hex chars from a KIM1 and 32 from an Arribada, a data line as 46 and 48.
// Exploring whether some other Arribada profile (AT+RCONF / AT+KMAC) can match
// the Kineis table is left for later.
//
// Returns the padded length, or 0 if the payload does not fit, which is worth
// logging rather than letting the module silently truncate it.
static size_t padForArribada(const char *in, char *out, size_t outSize) {
  if (in == nullptr || out == nullptr) return 0;

  size_t len = strlen(in);
  size_t padded = ((len + 7) / 8) * 8;      // round up to a whole 4-byte word

  if (padded > SAT_MAX_HEX_ARRIBADA || padded + 1 > outSize) return 0;

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

  // Ask the module to name itself. Both firmwares answer AT+FW=? and only the
  // KIM1 puts its own name in the reply, so one command separates them:
  //   KIM1 FW 2.1: "+FW=KIM1_V2.1"
  //   Arribada:    "+FW=5ad8cd5_Tx_gui_basic_Mp,v10.0.0_...,Oct 13 2025_08:10:51"
  //
  // Three more obvious-looking discriminators were tried and all three are
  // wrong, measured on both modules:
  //   - AT+PING is not Arribada-only; the KIM1 answers +OK (integration manual
  //     v2.3 section 3.3.3.b).
  //   - The AT+ID reply shape matches: KIM1 says "+ID=276f235", Arribada says
  //     "+ID=294848". Not the "+ID:28,..." the old KIM.h comments describe.
  //   - The "?" vs "=?" syntax split is the opposite way round from the
  //     Arribada wiki: this firmware wants "AT+FW=?" and rejects "AT+FW?"
  //     with +ERROR=1203, exactly like the KIM1.
  char reply[96];   // the Arribada firmware string is long

  kimSerial.begin(KIMBaud, SERIAL_8N1, RX_KIM, TX_KIM);
  delay(50);

  // Try the "=?" form first since both current firmwares use it, then the "?"
  // form the Arribada wiki documents, so a future build that switches to it
  // still gets recognised.
  probeAT("AT+FW=?", reply, sizeof(reply), 1500);
  if (strncmp(reply, "+FW=", 4) != 0) {
    probeAT("AT+FW?", reply, sizeof(reply), 1500);
  }

  if (strncmp(reply, "+FW=", 4) == 0) {
    // Only the KIM1 puts its own name in the version string. Anything else that
    // answers AT+FW at all is taken to be the Arribada, so its version string
    // can change freely - which it will, it carries a git hash and a build date.
    detectedType = (strstr(reply, "KIM") != nullptr) ? SAT_KIM1 : SAT_ARRIBADA;
  } else {
    // Nothing sensible came back from either form. Fall back to commands that
    // exist on one module only: AT+AFMT on the KIM1, AT+KMAC on the Arribada.
    // Each answers +ERROR=1203/+ERROR on the other.
    probeAT("AT+AFMT=?", reply, sizeof(reply), 1500);
    if (strncmp(reply, "+AFMT=", 6) == 0) {
      detectedType = SAT_KIM1;
    } else {
      probeAT("AT+KMAC=?", reply, sizeof(reply), 1500);
      if (strncmp(reply, "+KMAC=", 6) == 0) detectedType = SAT_ARRIBADA;
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
      // No AT+AFMT here; the equivalent knob is the MAC profile, and that is
      // set before every transmission rather than once per session because the
      // module forgets it whenever it loses power. Setting it now as well only
      // reports early whether the module is answering.
      if (Arribada.set_KMAC() != OK_ARRIBADA) {
        writeLogFile("ARRIBADA KMAC_ERR at configuration");
        return false;
      }
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
      if (len > SAT_MAX_HEX_KIM) {
        writeLogFile("SAT MSG_ERR: payload too long for KIM1 (" + String(len) + " hex chars, max " + String(SAT_MAX_HEX_KIM) + ")");
        return false;
      }
      return KIM.send_data((char *)hexPayload, len) == OK_KIM;

    case SAT_ARRIBADA: {
      // Re-select the MAC profile first. The module boots with +KMAC=0 and
      // refuses to transmit in that state (+ERROR=253), the setting does not
      // survive a power cut, and goToSleep() drops GPIO13 between messages -
      // so once per session is not enough, it has to be here.
      if (Arribada.set_KMAC() != OK_ARRIBADA) {
        writeLogFile("ARRIBADA KMAC_ERR - transmission will be refused");
      }

      char padded[SAT_MAX_HEX_ARRIBADA + 1];
      size_t paddedLen = padForArribada(hexPayload, padded, sizeof(padded));
      if (paddedLen == 0) {
        writeLogFile("SAT MSG_ERR: payload too long for ARRIBADA (" + String(len) + " hex chars, max " + String(SAT_MAX_HEX_ARRIBADA) + ")");
        return false;
      }
      if (Arribada.send_data(padded, paddedLen) == OK_ARRIBADA) return true;
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
