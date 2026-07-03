// -------------------------------------------------------------------------- //
// Pure extractions of the remaining small, self-contained helpers from
// popup-buoy.ino: the "row:count"/"index:data"/"name=value" line splitters
// used to parse SD-card files, the seconds->h/m/s converter, and the
// EEPROM 16-bit coverage-duration byte packing.
// -------------------------------------------------------------------------- //
#pragma once

#include <cstdint>
#include <string>

namespace buoy_logic {

struct RowCount {
  bool ok = false;
  int row = 0;
  int count = 0;
};

// Mirrors splitLineProgressFile(): "<row>:<count>".
RowCount splitLineProgressFile(const std::string &line);

struct IndexData {
  bool ok = false;
  int index = 0;
  std::string data;
};

// Mirrors splitLineDataFile(): "<index>:<data>".
IndexData splitLineDataFile(const std::string &line);

struct NameValue {
  bool ok = false;
  std::string name;
  int value = 0;
};

// Mirrors splitLineSuccessFile() (also used to parse /conf.txt):
// "<name>=<value>".
NameValue splitLineSuccessFile(const std::string &line);

struct HMS {
  int hours = 0;
  int minutes = 0;
  int seconds = 0;
};

// Mirrors ChangeSecondsInHoursAndMinutes().
HMS secondsToHoursMinutesSeconds(int totalSeconds);

// Mirrors the read side of eepromSaveTimeCoverage()/its callers:
//   Decimal_CoverageDuration = (EEPROM.read(2) << 8) | EEPROM.read(3);
uint16_t unpackCoverageDuration(uint8_t highByte, uint8_t lowByte);

// Mirrors the write side of eepromSaveTimeCoverage():
//   EEPROM.write(2, (timeCoverage >> 8) & 0xFF);
//   EEPROM.write(3, timeCoverage & 0xFF);
struct PackedCoverageDuration {
  uint8_t highByte;
  uint8_t lowByte;
};
PackedCoverageDuration packCoverageDuration(int timeCoverage);

}  // namespace buoy_logic
