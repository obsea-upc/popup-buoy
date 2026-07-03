// -------------------------------------------------------------------------- //
// Pure, Arduino-independent parsing logic extracted from popup-buoy.ino's
// sendHttpGetRequest() response handlers, so it can be unit tested on the
// host machine (test/native/) without an ESP32 toolchain.
//
// These mirror parseTimeResponse / parsePermissionResponse /
// parseSyncTimeResponse in popup-buoy.ino byte-for-byte in parsing
// behavior, MINUS the writeLogFile(...) side effects (which touch the SD
// card and don't belong in pure logic) -- callers get a structured result
// and decide what to log themselves.
// -------------------------------------------------------------------------- //
#pragma once

#include <string>

namespace buoy_logic {

enum class ReleaseMode { FRM, DM };

struct TimeParseResult {
  bool ok = false;
  std::string error;  // set when ok == false
  int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
};

// Mirrors parseTimeResponse(). Expects a payload shaped like:
//   {"success": true, "current_time": {"year": 2024, "month": 10, "day": 10,
//    "hour": 10, "minute": 40, "second": 34}}
TimeParseResult parseTimeResponse(const std::string &payload);

struct PermissionParseResult {
  bool ok = false;
  std::string error;  // set when ok == false
  int releaseFlag = 0;
  ReleaseMode releaseMode = ReleaseMode::DM;
  bool releaseModeWasUnknown = false;  // true if the payload's releaseMode
                                        // string wasn't "FRM"/"DM" and we
                                        // fell back to DM (mirrors the
                                        // original function's documented
                                        // fallback-and-log behavior)
  std::string rawReleaseModeValue;     // the as-parsed (still-quoted) releaseMode
                                        // value, e.g. "\"GARBAGE\"" -- only
                                        // meaningful when releaseModeWasUnknown,
                                        // kept so callers can reproduce the
                                        // original's "Unknown releaseMode: X"
                                        // log message exactly
  int sleeptime_h = 0;
  int sleeptime_m = 0;
};

// Mirrors parsePermissionResponse().
PermissionParseResult parsePermissionResponse(const std::string &payload);

struct SyncTimeParseResult {
  bool ok = false;
  int syncTime = 0;
};

// Mirrors parseSyncTimeResponse().
SyncTimeParseResult parseSyncTimeResponse(const std::string &payload);

}  // namespace buoy_logic
