// Native unit tests for logic/parsers.{h,cpp} -- the pure extraction of
// popup-buoy.ino's HTTP JSON response parsers.
#include <cstring>

#include "framework.h"
#include "parsers.h"

using buoy_logic::parsePermissionResponse;
using buoy_logic::parseSyncTimeResponse;
using buoy_logic::parseTimeResponse;
using buoy_logic::ReleaseMode;

// -------------------------------------------------------------------------- //
// parseTimeResponse
// -------------------------------------------------------------------------- //

TEST(time_wellformed_payload_parses_all_fields) {
  auto r = parseTimeResponse(
      "{\"success\": true, \"current_time\": {\"year\": 2024, \"month\": 10, "
      "\"day\": 10, \"hour\": 10, \"minute\": 40, \"second\": 34}}");
  CHECK(r.ok);
  CHECK_INT_EQ(r.year, 2024);
  CHECK_INT_EQ(r.month, 10);
  CHECK_INT_EQ(r.day, 10);
  CHECK_INT_EQ(r.hour, 10);
  CHECK_INT_EQ(r.minute, 40);
  CHECK_INT_EQ(r.second, 34);
}

TEST(time_missing_success_flag_fails) {
  auto r = parseTimeResponse("{\"current_time\": {\"year\": 2024}}");
  CHECK(!r.ok);
}

TEST(time_explicit_success_false_fails) {
  auto r = parseTimeResponse("{\"success\": false}");
  CHECK(!r.ok);
}

TEST(time_missing_each_field_individually_fails) {
  CHECK(!parseTimeResponse("{\"success\": true, \"month\":1,\"day\":1,\"hour\":0,\"minute\":0,\"second\":0}").ok);
  CHECK(!parseTimeResponse("{\"success\": true, \"year\":2024,\"day\":1,\"hour\":0,\"minute\":0,\"second\":0}").ok);
  CHECK(!parseTimeResponse("{\"success\": true, \"year\":2024,\"month\":1,\"hour\":0,\"minute\":0,\"second\":0}").ok);
  CHECK(!parseTimeResponse("{\"success\": true, \"year\":2024,\"month\":1,\"day\":1,\"minute\":0,\"second\":0}").ok);
  CHECK(!parseTimeResponse("{\"success\": true, \"year\":2024,\"month\":1,\"day\":1,\"hour\":0,\"second\":0}").ok);
  CHECK(!parseTimeResponse("{\"success\": true, \"year\":2024,\"month\":1,\"day\":1,\"hour\":0,\"minute\":0}").ok);
}

TEST(time_truncated_payload_fails_gracefully_not_crash) {
  auto r = parseTimeResponse("{\"success\": true, \"year\":");
  CHECK(!r.ok);
}

// -------------------------------------------------------------------------- //
// parsePermissionResponse
// -------------------------------------------------------------------------- //

static std::string permissionPayload(int flag, const char *mode, int h, int m) {
  return std::string("{\"success\": true, \"popup_id\": \"1\", \"permission\": {\"releaseFlag\": ") +
         std::to_string(flag) + ", \"releaseMode\": \"" + mode +
         "\", \"sleeptime_h\": \"" + std::to_string(h) + "\", \"sleeptime_m\": \"" +
         std::to_string(m) + "\"}}";
}

TEST(permission_release_flag_zero_and_one) {
  CHECK_INT_EQ(parsePermissionResponse(permissionPayload(0, "DM", 24, 0)).releaseFlag, 0);
  CHECK_INT_EQ(parsePermissionResponse(permissionPayload(1, "DM", 24, 0)).releaseFlag, 1);
}

TEST(permission_release_mode_frm_and_dm) {
  auto frm = parsePermissionResponse(permissionPayload(1, "FRM", 24, 0));
  CHECK(frm.ok);
  CHECK(frm.releaseMode == ReleaseMode::FRM);
  CHECK(!frm.releaseModeWasUnknown);

  auto dm = parsePermissionResponse(permissionPayload(1, "DM", 24, 0));
  CHECK(dm.ok);
  CHECK(dm.releaseMode == ReleaseMode::DM);
}

TEST(permission_release_mode_unknown_falls_back_to_dm_and_flags_it) {
  auto r = parsePermissionResponse(permissionPayload(1, "GARBAGE", 24, 0));
  CHECK(r.ok);
  CHECK(r.releaseMode == ReleaseMode::DM);
  CHECK(r.releaseModeWasUnknown);
  CHECK(r.rawReleaseModeValue == "\"GARBAGE\"");
}

TEST(permission_sleeptimes_parsed) {
  auto r = parsePermissionResponse(permissionPayload(0, "DM", 12, 30));
  CHECK(r.ok);
  CHECK_INT_EQ(r.sleeptime_h, 12);
  CHECK_INT_EQ(r.sleeptime_m, 30);
}

TEST(permission_missing_release_flag_fails) {
  CHECK(!parsePermissionResponse("{\"success\": true, \"releaseMode\": \"DM\"}").ok);
}

TEST(permission_missing_release_mode_fails) {
  CHECK(!parsePermissionResponse("{\"success\": true, \"releaseFlag\": 1}").ok);
}

TEST(permission_success_false_with_message_is_a_known_bug_dropping_the_message) {
  // KNOWN BUG (faithfully ported from popup-buoy.ino's parsePermissionResponse):
  // `messageStart += 10` was meant to skip past the `"message":"` key
  // (11 characters: " m e s s a g e " : "), but 10 lands ON the opening
  // quote of the value instead of just after it. The immediately-following
  // `payload.indexOf("\"", messageStart)` then finds that very same quote
  // (indexOf includes the start position), so messageEnd == messageStart
  // and the extracted message is always an empty string -- the server's
  // actual failure reason is silently dropped from the log every time.
  // This doesn't affect release decision logic (only the logged text), but
  // is a real bug worth fixing (change +10 to +11) -- documented here
  // rather than fixed, since fixing behavior wasn't in scope for this
  // extraction. See docs/ARCHITECTURE.md Known rough edges.
  auto r = parsePermissionResponse("{\"success\": false, \"message\":\"popup_id 9 not found\"}");
  CHECK(!r.ok);
  CHECK(r.error == "Request failed: ");
  CHECK(r.error.find("popup_id 9 not found") == std::string::npos);
}

TEST(permission_success_false_without_message_reports_unknown_error) {
  auto r = parsePermissionResponse("{\"success\": false}");
  CHECK(!r.ok);
  CHECK(r.error.find("Unknown error") != std::string::npos);
}

// -------------------------------------------------------------------------- //
// parseSyncTimeResponse
// -------------------------------------------------------------------------- //

TEST(synctime_present_parses) {
  auto r = parseSyncTimeResponse("{\"sync_time\": 9}");
  CHECK(r.ok);
  CHECK_INT_EQ(r.syncTime, 9);
}

TEST(synctime_missing_key_fails) {
  auto r = parseSyncTimeResponse("{}");
  CHECK(!r.ok);
}

TEST(synctime_non_numeric_value_parses_as_zero_not_crash) {
  // Arduino String::toInt() (and our lenient std::atoi stand-in) returns 0
  // for a non-numeric leading value rather than raising an error --
  // documents current (lenient, silently-wrong) behavior.
  auto r = parseSyncTimeResponse("{\"sync_time\": \"nine\"}");
  CHECK(r.ok);
  CHECK_INT_EQ(r.syncTime, 0);
}

int main(void) {
  RUN_TEST(time_wellformed_payload_parses_all_fields);
  RUN_TEST(time_missing_success_flag_fails);
  RUN_TEST(time_explicit_success_false_fails);
  RUN_TEST(time_missing_each_field_individually_fails);
  RUN_TEST(time_truncated_payload_fails_gracefully_not_crash);
  RUN_TEST(permission_release_flag_zero_and_one);
  RUN_TEST(permission_release_mode_frm_and_dm);
  RUN_TEST(permission_release_mode_unknown_falls_back_to_dm_and_flags_it);
  RUN_TEST(permission_sleeptimes_parsed);
  RUN_TEST(permission_missing_release_flag_fails);
  RUN_TEST(permission_missing_release_mode_fails);
  RUN_TEST(permission_success_false_with_message_is_a_known_bug_dropping_the_message);
  RUN_TEST(permission_success_false_without_message_reports_unknown_error);
  RUN_TEST(synctime_present_parses);
  RUN_TEST(synctime_missing_key_fails);
  RUN_TEST(synctime_non_numeric_value_parses_as_zero_not_crash);
  TEST_REPORT_AND_EXIT();
}
