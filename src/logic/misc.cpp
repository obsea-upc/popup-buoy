#include "misc.h"

#include <cstdlib>

namespace buoy_logic {

namespace {
int lenientToInt(const std::string &s) { return std::atoi(s.c_str()); }
}  // namespace

RowCount splitLineProgressFile(const std::string &line) {
  RowCount r;
  size_t sep = line.find(':');
  if (sep == std::string::npos) return r;  // ok == false, matches original
                                            // leaving out-params untouched
  r.row = lenientToInt(line.substr(0, sep));
  r.count = lenientToInt(line.substr(sep + 1));
  r.ok = true;
  return r;
}

IndexData splitLineDataFile(const std::string &line) {
  IndexData r;
  size_t sep = line.find(':');
  if (sep == std::string::npos) return r;
  r.index = lenientToInt(line.substr(0, sep));
  r.data = line.substr(sep + 1);
  r.ok = true;
  return r;
}

NameValue splitLineSuccessFile(const std::string &line) {
  NameValue r;
  size_t sep = line.find('=');
  if (sep == std::string::npos) return r;
  r.name = line.substr(0, sep);
  r.value = lenientToInt(line.substr(sep + 1));
  r.ok = true;
  return r;
}

HMS secondsToHoursMinutesSeconds(int totalSeconds) {
  HMS r;
  r.hours = totalSeconds / 3600;
  r.minutes = (totalSeconds % 3600) / 60;
  r.seconds = (totalSeconds % 3600) % 60;
  return r;
}

uint16_t unpackCoverageDuration(uint8_t highByte, uint8_t lowByte) {
  return (static_cast<uint16_t>(highByte) << 8) | lowByte;
}

PackedCoverageDuration packCoverageDuration(int timeCoverage) {
  PackedCoverageDuration p;
  p.highByte = static_cast<uint8_t>((timeCoverage >> 8) & 0xFF);
  p.lowByte = static_cast<uint8_t>(timeCoverage & 0xFF);
  return p;
}

}  // namespace buoy_logic
