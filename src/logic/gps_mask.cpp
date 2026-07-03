#include "gps_mask.h"

#include <cmath>
#include <cstdio>

namespace buoy_logic {

std::string maskGPS(double gpsLat, double gpsLong, uint32_t epochTime, bool withAdc,
                     const std::string &adcHexOrCrcHex) {
  char hex_latitude[9], hex_longitude[9], hex_epochTime[9];

  if (gpsLat == 200 && gpsLong == 200) {
    std::snprintf(hex_latitude, sizeof(hex_latitude), "FFFFFFFF");
    std::snprintf(hex_longitude, sizeof(hex_longitude), "FFFFFFFF");
  } else {
    // Matches the original's `int latitude = gpsLat * pow(10,6);` (32-bit,
    // truncating). Using an explicit int32_t here (rather than the
    // original's bare `int`, and rather than the original's "unsigned
    // long" for the hex formatting) makes the width portable between the
    // 32-bit embedded target and a 64-bit host build without changing the
    // numeric result: lat/long * 1e6 always fits comfortably in 32 bits.
    int32_t latitude = static_cast<int32_t>(gpsLat * std::pow(10, 6));
    int32_t longitude = static_cast<int32_t>(gpsLong * std::pow(10, 6));

    uint32_t latBits = (latitude < 0) ? static_cast<uint32_t>(4294967296LL + latitude)
                                       : static_cast<uint32_t>(latitude);
    uint32_t longBits = (longitude < 0) ? static_cast<uint32_t>(4294967296LL + longitude)
                                         : static_cast<uint32_t>(longitude);

    std::snprintf(hex_latitude, sizeof(hex_latitude), "%08X", latBits);
    std::snprintf(hex_longitude, sizeof(hex_longitude), "%08X", longBits);
  }

  std::snprintf(hex_epochTime, sizeof(hex_epochTime), "%08X", epochTime);

  std::string maskedData = std::string(hex_latitude) + hex_longitude + hex_epochTime;

  (void)withAdc;  // both branches append the same caller-provided 2-hex-char
                   // suffix; kept as a parameter to document which of the
                   // original's two #ifdef WORK_ADC branches it stands in for
  return maskedData + adcHexOrCrcHex;
}

}  // namespace buoy_logic
