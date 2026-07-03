#include "parsers.h"

#include <cstdlib>

namespace buoy_logic {

namespace {

// Arduino's String::toInt() is documented to behave like atol(): parse
// optional leading whitespace/sign then digits, stop at the first invalid
// character, return 0 if nothing valid was found. std::atoi has the same
// contract (no exceptions), so it's a faithful native stand-in.
int lenientToInt(const std::string &s) { return std::atoi(s.c_str()); }

}  // namespace

TimeParseResult parseTimeResponse(const std::string &payload) {
  TimeParseResult r;

  if (payload.find("\"success\": true") == std::string::npos) {
    r.error = "Request failed: Time parsing failed due to missing success flag.";
    return r;
  }

  auto extractField = [&](const char *key, char terminator, int &out, const char *missingMsg) -> bool {
    size_t start = payload.find(key);
    if (start == std::string::npos) {
      r.error = missingMsg;
      return false;
    }
    start = payload.find(':', start) + 1;
    size_t end = payload.find(terminator, start);
    std::string field = (end == std::string::npos) ? payload.substr(start) : payload.substr(start, end - start);
    out = lenientToInt(field);
    return true;
  };

  if (!extractField("\"year\":", ',', r.year, "Missing year in response")) return r;
  if (!extractField("\"month\":", ',', r.month, "Missing month in response")) return r;
  if (!extractField("\"day\":", ',', r.day, "Missing day in response")) return r;
  if (!extractField("\"hour\":", ',', r.hour, "Missing hour in response")) return r;
  if (!extractField("\"minute\":", ',', r.minute, "Missing minute in response")) return r;
  if (!extractField("\"second\":", '}', r.second, "Missing second in response")) return r;

  r.ok = true;
  return r;
}

PermissionParseResult parsePermissionResponse(const std::string &payload) {
  PermissionParseResult r;

  if (payload.find("\"success\": true") == std::string::npos) {
    size_t messageStart = payload.find("\"message\":\"");
    if (messageStart != std::string::npos) {
      messageStart += 10;  // matches original's `messageStart += 10` (points
                            // mid-way into the opening quote, faithfully
                            // reproduced from popup-buoy.ino)
      size_t messageEnd = payload.find('"', messageStart);
      std::string message = (messageEnd == std::string::npos)
                                 ? payload.substr(messageStart)
                                 : payload.substr(messageStart, messageEnd - messageStart);
      r.error = "Request failed: " + message;
    } else {
      r.error = "Request failed: Unknown error";
    }
    return r;
  }

  size_t flagStart = payload.find("\"releaseFlag\":");
  if (flagStart == std::string::npos) {
    r.error = "Missing releaseFlag in response";
    return r;
  }
  flagStart = payload.find(':', flagStart) + 1;
  size_t flagEnd = payload.find(',', flagStart);
  std::string flagStr = (flagEnd == std::string::npos) ? payload.substr(flagStart) : payload.substr(flagStart, flagEnd - flagStart);
  r.releaseFlag = lenientToInt(flagStr);

  size_t modeStart = payload.find("\"releaseMode\": ");
  if (modeStart == std::string::npos) {
    r.error = "Missing releaseMode in response";
    return r;
  }
  modeStart = payload.find(':', modeStart) + 2;  // +2 to include the opening quote, matches original
  size_t modeEnd = payload.find(',', modeStart);
  std::string releaseModeStr = (modeEnd == std::string::npos) ? payload.substr(modeStart) : payload.substr(modeStart, modeEnd - modeStart);
  if (releaseModeStr == "\"FRM\"") {
    r.releaseMode = ReleaseMode::FRM;
  } else if (releaseModeStr == "\"DM\"") {
    r.releaseMode = ReleaseMode::DM;
  } else {
    r.releaseMode = ReleaseMode::DM;
    r.releaseModeWasUnknown = true;
    r.rawReleaseModeValue = releaseModeStr;
  }

  size_t sleepHStart = payload.find("\"sleeptime_h\":");
  if (sleepHStart == std::string::npos) {
    r.error = "Missing sleeptime_h in response";
    return r;
  }
  sleepHStart = payload.find(':', sleepHStart) + 3;  // matches original's `+3`
  size_t sleepHEnd = payload.find(',', sleepHStart);
  std::string sleepHStr = (sleepHEnd == std::string::npos)
                               ? payload.substr(sleepHStart)
                               : payload.substr(sleepHStart, (sleepHEnd - 1) - sleepHStart);
  r.sleeptime_h = lenientToInt(sleepHStr);

  size_t sleepMStart = payload.find("\"sleeptime_m\":");
  if (sleepMStart == std::string::npos) {
    r.error = "Missing sleeptime_m in response";
    return r;
  }
  sleepMStart = payload.find(':', sleepMStart) + 3;
  size_t sleepMEnd = payload.find('}', sleepMStart);
  std::string sleepMStr = (sleepMEnd == std::string::npos)
                               ? payload.substr(sleepMStart)
                               : payload.substr(sleepMStart, (sleepMEnd - 1) - sleepMStart);
  r.sleeptime_m = lenientToInt(sleepMStr);

  r.ok = true;
  return r;
}

SyncTimeParseResult parseSyncTimeResponse(const std::string &payload) {
  SyncTimeParseResult r;
  size_t syncTimeStart = payload.find("\"sync_time\":");
  if (syncTimeStart == std::string::npos) {
    return r;  // ok == false
  }
  syncTimeStart += 12;
  size_t end = payload.find('}', syncTimeStart);
  std::string field = (end == std::string::npos) ? payload.substr(syncTimeStart) : payload.substr(syncTimeStart, end - syncTimeStart);
  r.syncTime = lenientToInt(field);
  r.ok = true;
  return r;
}

}  // namespace buoy_logic
