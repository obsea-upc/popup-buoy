// Native unit tests for logic/gps_mask.{h,cpp} -- the pure extraction of
// popup-buoy.ino's maskGPS().
#include "framework.h"
#include "gps_mask.h"

using buoy_logic::maskGPS;

TEST(no_fix_sentinel_encodes_as_all_f) {
  std::string msg = maskGPS(200.0, 200.0, 0x12345678u, true, "AB");
  CHECK(msg.substr(0, 8) == "FFFFFFFF");
  CHECK(msg.substr(8, 8) == "FFFFFFFF");
}

TEST(positive_lat_long_encode_as_plain_hex_of_microdegrees) {
  // 41.2 deg -> 41,200,000 microdegrees -> 0x0274A980
  std::string msg = maskGPS(41.2, 2.1, 0, true, "00");
  CHECK(msg.substr(0, 8) == "0274A980");
  // 2.1 deg -> 2,100,000 -> 0x00200B20
  CHECK(msg.substr(8, 8) == "00200B20");
}

TEST(negative_lat_long_encode_as_32bit_twos_complement) {
  // -41.2 deg -> -41,200,000 -> 2^32 - 41,200,000 = 0xFD8B5680
  // -2.1 deg  -> -2,100,000  -> 2^32 - 2,100,000  = 0xFFDFF4E0
  std::string msg = maskGPS(-41.2, -2.1, 0, true, "00");
  CHECK(msg.substr(0, 8) == "FD8B5680");
  CHECK(msg.substr(8, 8) == "FFDFF4E0");
}

TEST(epoch_zero_and_max_encode_correctly) {
  std::string msgZero = maskGPS(0.0, 0.0, 0u, true, "00");
  CHECK(msgZero.substr(16, 8) == "00000000");

  std::string msgMax = maskGPS(0.0, 0.0, 0xFFFFFFFFu, true, "00");
  CHECK(msgMax.substr(16, 8) == "FFFFFFFF");
}

TEST(adc_suffix_is_appended_verbatim) {
  std::string msg = maskGPS(0.0, 0.0, 0u, true, "3F");
  CHECK(msg.size() == 26);  // 8 + 8 + 8 + 2
  CHECK(msg.substr(24, 2) == "3F");
}

TEST(message_length_is_always_26_hex_chars) {
  CHECK(maskGPS(200.0, 200.0, 0u, true, "00").size() == 26);
  CHECK(maskGPS(-89.9, 179.9, 1700000000u, true, "FF").size() == 26);
}

int main(void) {
  RUN_TEST(no_fix_sentinel_encodes_as_all_f);
  RUN_TEST(positive_lat_long_encode_as_plain_hex_of_microdegrees);
  RUN_TEST(negative_lat_long_encode_as_32bit_twos_complement);
  RUN_TEST(epoch_zero_and_max_encode_correctly);
  RUN_TEST(adc_suffix_is_appended_verbatim);
  RUN_TEST(message_length_is_always_26_hex_chars);
  TEST_REPORT_AND_EXIT();
}
