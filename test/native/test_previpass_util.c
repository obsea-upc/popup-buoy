/*
 * Native unit tests for previpass_util.c -- pure C, no Arduino/hardware
 * dependency, compiled and run on the host machine.
 */
#include <string.h>

#include "framework.h"
#include "previpass_util.h"

static void roundtrip(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s) {
  struct CalendarDateTime_t in = {y, mo, d, h, mi, s};
  uint32_t sec90;
  PREVIPASS_UTIL_date_calendar_stu90(in, &sec90);

  struct CalendarDateTime_t out;
  PREVIPASS_UTIL_date_stu90_calendar(sec90, &out);

  CHECK_INT_EQ(out.gpsYear, in.gpsYear);
  CHECK_INT_EQ(out.gpsMonth, in.gpsMonth);
  CHECK_INT_EQ(out.gpsDay, in.gpsDay);
  CHECK_INT_EQ(out.gpsHour, in.gpsHour);
  CHECK_INT_EQ(out.gpsMinute, in.gpsMinute);
  CHECK_INT_EQ(out.gpsSecond, in.gpsSecond);
}

TEST(date_epoch_1990_01_01_is_sec90_zero) {
  struct CalendarDateTime_t epoch = {1990, 1, 1, 0, 0, 0};
  uint32_t sec90 = 12345;  // poison, must be overwritten
  PREVIPASS_UTIL_date_calendar_stu90(epoch, &sec90);
  CHECK_INT_EQ(sec90, 0);
}

TEST(date_roundtrip_epoch) { roundtrip(1990, 1, 1, 0, 0, 0); }

TEST(date_roundtrip_y2k) { roundtrip(2000, 1, 1, 0, 0, 0); }

TEST(date_roundtrip_leap_day) { roundtrip(2024, 2, 29, 12, 0, 0); }

TEST(date_roundtrip_year_boundary) {
  // NOTE: does NOT include a December date on purpose -- every December
  // date hits a real out-of-bounds read in PREVIPASS_UTIL_date_stu90_calendar
  // (see test_previpass_known_issues.c), which aborts this ASan-instrumented
  // binary. Jan 1st alone still exercises the Dec-31->Jan-1 rollover from
  // the *encoding* side (date_calendar_stu90), which is not affected.
  roundtrip(2025, 1, 1, 0, 0, 0);
}

TEST(date_roundtrip_ordinary_datetime) { roundtrip(2024, 6, 15, 8, 30, 45); }

TEST(date_roundtrip_day_after_non_leap_february) { roundtrip(2023, 3, 1, 0, 0, 0); }

TEST(date_stu90_to_calendar_matches_calendar_to_stu90_ordering) {
  // A date one day later must convert to a sec90 exactly 86400 greater.
  struct CalendarDateTime_t day1 = {2024, 5, 10, 0, 0, 0};
  struct CalendarDateTime_t day2 = {2024, 5, 11, 0, 0, 0};
  uint32_t sec1, sec2;
  PREVIPASS_UTIL_date_calendar_stu90(day1, &sec1);
  PREVIPASS_UTIL_date_calendar_stu90(day2, &sec2);
  CHECK_INT_EQ(sec2 - sec1, 86400);
}

TEST(elevation_distance_increases_with_elevation) {
  // Higher required elevation -> stricter (smaller) visibility footprint
  // -> smaller squared distance threshold, for a fixed orbit altitude.
  float axis = 7226.13f;
  float d0 = PREVIPASS_UTIL_sat_elevation_distance2(0.0f, axis);
  float d45 = PREVIPASS_UTIL_sat_elevation_distance2(45.0f, axis);
  float d80 = PREVIPASS_UTIL_sat_elevation_distance2(80.0f, axis);
  CHECK(d0 > d45);
  CHECK(d45 > d80);
}

TEST(elevation_distance_is_positive_for_realistic_leo_orbit) {
  float d = PREVIPASS_UTIL_sat_elevation_distance2(5.0f, 7226.13f);
  CHECK(d > 0.0f);
}

int main(void) {
  RUN_TEST(date_epoch_1990_01_01_is_sec90_zero);
  RUN_TEST(date_roundtrip_epoch);
  RUN_TEST(date_roundtrip_y2k);
  RUN_TEST(date_roundtrip_leap_day);
  RUN_TEST(date_roundtrip_year_boundary);
  RUN_TEST(date_roundtrip_ordinary_datetime);
  RUN_TEST(date_roundtrip_day_after_non_leap_february);
  RUN_TEST(date_stu90_to_calendar_matches_calendar_to_stu90_ordering);
  RUN_TEST(elevation_distance_increases_with_elevation);
  RUN_TEST(elevation_distance_is_positive_for_realistic_leo_orbit);
  TEST_REPORT_AND_EXIT();
}
