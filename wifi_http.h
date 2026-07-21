#pragma once
#include <Arduino.h>

// Server request types (map to the pop-up server HTTP endpoints).
enum ActionType {
  RELEASE = 1,
  PERMISSION = 2,
  GETTIME = 3,
  GETSYNCTIME = 4
};

// Buoy surface behaviour requested by the server.
enum ReleaseMode {
  FRM,  // FAST RECOVERY MODE
  DM    // DRIFTING MODE
};

// Connect to the pop-up server's Wi-Fi AP. Returns true on success.
bool connectToRaspWiFi();
String getWiFiFailureReason(int status);

// Issue an HTTP GET to the server for the given action, then parse the response.
// GETTIME fills the year_lander..second_lander globals; GETSYNCTIME fills syncTime;
// PERMISSION fills releaseFlag/releaseMode/sleeptime_h/sleeptime_m via the refs.
bool sendHttpGetRequest(int idBoia, ActionType action, int &releaseFlag, ReleaseMode &releaseMode, int &sleeptime_h, int &sleeptime_m);

// Response parsers for each endpoint.
bool parseTimeResponse(const String &payload, int &year, int &month, int &day, int &hour, int &minute, int &second);
bool parsePermissionResponse(const String &payload, int &releaseFlag, ReleaseMode &releaseMode, int &sleeptime_h, int &sleeptime_m);
bool parseSyncTimeResponse(const String& payload);
