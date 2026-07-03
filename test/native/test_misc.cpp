// Native unit tests for logic/misc.{h,cpp}: line splitters, seconds->h/m/s
// conversion, EEPROM coverage-duration byte packing.
#include "framework.h"
#include "misc.h"

using namespace buoy_logic;

// -------------------------------------------------------------------------- //
// splitLineProgressFile: "<row>:<count>"
// -------------------------------------------------------------------------- //

TEST(progress_line_normal) {
  auto r = splitLineProgressFile("12:3");
  CHECK(r.ok);
  CHECK_INT_EQ(r.row, 12);
  CHECK_INT_EQ(r.count, 3);
}

TEST(progress_line_missing_separator_is_not_ok) {
  CHECK(!splitLineProgressFile("123").ok);
}

TEST(progress_line_empty_string_is_not_ok) { CHECK(!splitLineProgressFile("").ok); }

TEST(progress_line_separator_at_start) {
  auto r = splitLineProgressFile(":5");
  CHECK(r.ok);
  CHECK_INT_EQ(r.row, 0);  // atoi("") == 0
  CHECK_INT_EQ(r.count, 5);
}

TEST(progress_line_separator_at_end) {
  auto r = splitLineProgressFile("7:");
  CHECK(r.ok);
  CHECK_INT_EQ(r.row, 7);
  CHECK_INT_EQ(r.count, 0);
}

// -------------------------------------------------------------------------- //
// splitLineDataFile: "<index>:<data>"
// -------------------------------------------------------------------------- //

TEST(data_line_normal) {
  auto r = splitLineDataFile("42:0123456789ABCDEF");
  CHECK(r.ok);
  CHECK_INT_EQ(r.index, 42);
  CHECK(r.data == "0123456789ABCDEF");
}

TEST(data_line_missing_separator_is_not_ok) { CHECK(!splitLineDataFile("nocolon").ok); }

// -------------------------------------------------------------------------- //
// splitLineSuccessFile (also used for /conf.txt): "<name>=<value>"
// -------------------------------------------------------------------------- //

TEST(conf_line_normal) {
  auto r = splitLineSuccessFile("idBuoy=3");
  CHECK(r.ok);
  CHECK(r.name == "idBuoy");
  CHECK_INT_EQ(r.value, 3);
}

TEST(conf_line_missing_separator_is_not_ok) { CHECK(!splitLineSuccessFile("garbage").ok); }

// -------------------------------------------------------------------------- //
// secondsToHoursMinutesSeconds
// -------------------------------------------------------------------------- //

TEST(hms_zero) {
  auto r = secondsToHoursMinutesSeconds(0);
  CHECK_INT_EQ(r.hours, 0);
  CHECK_INT_EQ(r.minutes, 0);
  CHECK_INT_EQ(r.seconds, 0);
}

TEST(hms_exact_hour_boundary) {
  auto r = secondsToHoursMinutesSeconds(3600);
  CHECK_INT_EQ(r.hours, 1);
  CHECK_INT_EQ(r.minutes, 0);
  CHECK_INT_EQ(r.seconds, 0);
}

TEST(hms_exact_minute_boundary) {
  auto r = secondsToHoursMinutesSeconds(60);
  CHECK_INT_EQ(r.hours, 0);
  CHECK_INT_EQ(r.minutes, 1);
  CHECK_INT_EQ(r.seconds, 0);
}

TEST(hms_mixed) {
  auto r = secondsToHoursMinutesSeconds(3725);  // 1h 2m 5s
  CHECK_INT_EQ(r.hours, 1);
  CHECK_INT_EQ(r.minutes, 2);
  CHECK_INT_EQ(r.seconds, 5);
}

TEST(hms_large_value_beyond_24h_is_not_capped) {
  // The buoy's callers are expected to cap sleep durations themselves
  // (MAX_SLEEP_TIME_s) before calling this -- it does no clamping itself.
  auto r = secondsToHoursMinutesSeconds(100000);  // 27h 46m 40s
  CHECK_INT_EQ(r.hours, 27);
  CHECK_INT_EQ(r.minutes, 46);
  CHECK_INT_EQ(r.seconds, 40);
}

// -------------------------------------------------------------------------- //
// EEPROM coverage-duration byte packing
// -------------------------------------------------------------------------- //

TEST(coverage_duration_pack_unpack_roundtrip_zero) {
  auto p = packCoverageDuration(0);
  CHECK_INT_EQ(unpackCoverageDuration(p.highByte, p.lowByte), 0);
}

TEST(coverage_duration_pack_unpack_roundtrip_max) {
  auto p = packCoverageDuration(0xFFFF);
  CHECK_INT_EQ(p.highByte, 0xFF);
  CHECK_INT_EQ(p.lowByte, 0xFF);
  CHECK_INT_EQ(unpackCoverageDuration(p.highByte, p.lowByte), 0xFFFF);
}

TEST(coverage_duration_byte_order_regression_guard) {
  // 0x0100 = 256: high byte must be 0x01, low byte 0x00. Swapping the byte
  // order here (a classic copy-paste mistake in the original's
  // `(EEPROM.read(2) << 8) | EEPROM.read(3)` / write-byte-2-then-3 pairing)
  // would silently produce 1 or a wildly wrong duration instead of 256.
  auto p = packCoverageDuration(0x0100);
  CHECK_INT_EQ(p.highByte, 0x01);
  CHECK_INT_EQ(p.lowByte, 0x00);
  CHECK_INT_EQ(unpackCoverageDuration(p.highByte, p.lowByte), 0x0100);
}

int main(void) {
  RUN_TEST(progress_line_normal);
  RUN_TEST(progress_line_missing_separator_is_not_ok);
  RUN_TEST(progress_line_empty_string_is_not_ok);
  RUN_TEST(progress_line_separator_at_start);
  RUN_TEST(progress_line_separator_at_end);
  RUN_TEST(data_line_normal);
  RUN_TEST(data_line_missing_separator_is_not_ok);
  RUN_TEST(conf_line_normal);
  RUN_TEST(conf_line_missing_separator_is_not_ok);
  RUN_TEST(hms_zero);
  RUN_TEST(hms_exact_hour_boundary);
  RUN_TEST(hms_exact_minute_boundary);
  RUN_TEST(hms_mixed);
  RUN_TEST(hms_large_value_beyond_24h_is_not_capped);
  RUN_TEST(coverage_duration_pack_unpack_roundtrip_zero);
  RUN_TEST(coverage_duration_pack_unpack_roundtrip_max);
  RUN_TEST(coverage_duration_byte_order_regression_guard);
  TEST_REPORT_AND_EXIT();
}
