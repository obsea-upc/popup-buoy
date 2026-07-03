// -------------------------------------------------------------------------- //
// Pure extraction of popup-buoy.ino's maskGPS(): encodes GPS lat/long/epoch
// (+ optionally battery ADC reading) into the fixed-width hex ARGOS message
// body. No Arduino dependency, no logging side effect (caller logs the
// result if it wants to, mirroring how the original also called
// writeLogFile with the encoded message).
// -------------------------------------------------------------------------- //
#pragma once

#include <cstdint>
#include <string>

namespace buoy_logic {

// Mirrors maskGPS(). gpsLat/gpsLong use the buoy firmware's sentinel value
// 200/200 to mean "no GPS fix" (see gpsSave() in popup-buoy.ino), which
// this function encodes as "FFFFFFFF"/"FFFFFFFF" exactly like the original.
//
// withAdc selects between the two compile-time branches in the original
// (#ifdef WORK_ADC): true appends the 2-hex-digit ADC reading (adcReadHex,
// expected to already be a 2-char hex string); false appends a 2-hex-digit
// CRC8 (crc8OfMaskedData) instead. The caller computes whichever one
// applies and passes it in -- CRC8 itself isn't reimplemented here (it's
// FastCRC, an Arduino library, not extracted).
std::string maskGPS(double gpsLat, double gpsLong, uint32_t epochTime, bool withAdc,
                     const std::string &adcHexOrCrcHex);

}  // namespace buoy_logic
