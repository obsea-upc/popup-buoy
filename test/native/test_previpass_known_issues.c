/*
 * Regression tests for real bugs found while writing this native test
 * suite. Kept separate from the main test files because they intentionally
 * exercise undefined behavior that AddressSanitizer treats as fatal --
 * this file is built with -fsanitize=undefined only (see run_tests.sh),
 * which reports the violation to stderr but does not abort, so the process
 * can keep running and the assertions below can still execute.
 *
 * DO NOT "fix" these tests by avoiding the buggy input to make them
 * quiet -- that defeats their purpose. If the underlying bug in
 * previpass_util.c is ever fixed, these tests documenting it should be
 * deleted (or converted to plain regression tests), not adjusted to dodge it.
 */
#include "framework.h"
#include "previpass_util.h"

// -------------------------------------------------------------------------- //
// KNOWN BUG: PREVIPASS_UTIL_date_stu90_calendar (previpass_util.c:196) reads
// __ek_quanti[dateTime->gpsMonth - 1][isLeapYear] BEFORE checking
// `dateTime->gpsMonth <= 12` in the loop condition:
//
//   while ((numberOfDays > __ek_quanti[dateTime->gpsMonth - 1][isLeapYear])
//           && (dateTime->gpsMonth <= 12))
//     ++(dateTime->gpsMonth);
//
// Because C only short-circuits *after* evaluating the first operand, the
// array read on the LHS happens every time the loop is entered with
// gpsMonth == 13, regardless of the RHS bound check -- which is exactly
// what happens for every date whose day-of-year falls in December (the
// loop always overshoots month 12 by one iteration before the `<= 12`
// check can stop it). __ek_quanti is a 12-row table (indices 0..11); index
// 12 reads one uint32_t[2] row past the end of the array.
//
// In this environment it happens to read adjacent-but-mapped memory and
// the final computed date still comes out correct (the OOB read's result
// is ANDed with the now-false `month <= 12`, so it doesn't change control
// flow) -- but it is still a real out-of-bounds global read on every
// December date, is undefined behavior, and is not guaranteed to stay
// harmless (different compiler, optimization level, or memory layout on
// the actual ESP32 target could behave differently).
//
// See docs/ARCHITECTURE.md -- this should be added to Known rough edges,
// and reported upstream to the Kineis PREVIPASS library maintainers since
// previpass.c/previpass_util.c are vendored, not buoy-project code.
// -------------------------------------------------------------------------- //

static void assert_roundtrips_despite_the_bug(uint16_t y, uint8_t mo, uint8_t d) {
  struct CalendarDateTime_t in = {y, mo, d, 0, 0, 0};
  uint32_t sec90;
  PREVIPASS_UTIL_date_calendar_stu90(in, &sec90);

  struct CalendarDateTime_t out;
  PREVIPASS_UTIL_date_stu90_calendar(sec90, &out);  // <-- triggers the OOB read

  // Document that, today, in this environment, the *result* still happens
  // to be correct despite the out-of-bounds read.
  CHECK_INT_EQ(out.gpsYear, in.gpsYear);
  CHECK_INT_EQ(out.gpsMonth, in.gpsMonth);
  CHECK_INT_EQ(out.gpsDay, in.gpsDay);
}

TEST(every_december_date_triggers_the_out_of_bounds_read) {
  assert_roundtrips_despite_the_bug(2024, 12, 1);
  assert_roundtrips_despite_the_bug(2024, 12, 15);
  assert_roundtrips_despite_the_bug(2024, 12, 31);
  assert_roundtrips_despite_the_bug(2023, 12, 31);  // non-leap year too
}

TEST(november_30_does_not_trigger_the_bug) {
  // Sanity control: confirms the bug is specifically a December-boundary
  // issue, not present throughout the whole year.
  assert_roundtrips_despite_the_bug(2024, 11, 30);
}

int main(void) {
  RUN_TEST(every_december_date_triggers_the_out_of_bounds_read);
  RUN_TEST(november_30_does_not_trigger_the_bug);
  TEST_REPORT_AND_EXIT();
}
