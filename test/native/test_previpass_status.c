/*
 * Native unit tests for the constellation status conversion functions in
 * previpass.c (format A <-> generic, format B <-> generic, generic ->
 * format A). Exhaustive over the small documented enums in previpass.h.
 */
#include "framework.h"
#include "previpass.h"

// -------------------------------------------------------------------------- //
// Format A -> generic
// -------------------------------------------------------------------------- //

static void check_format_a_downlink(enum SatDownlinkStatusFormatA_t in, SatHexId_t sat,
                                     enum SatDownlinkStatus_t expected) {
  enum SatDownlinkStatus_t dl;
  enum SatUplinkStatus_t ul;
  PREVIPASS_status_format_a_to_generic(in, FMT_A_SAT_UPLK_OUT_OF_SERVICE, sat, &dl, &ul);
  CHECK_INT_EQ(dl, expected);
}

TEST(format_a_downlink_table) {
  check_format_a_downlink(FMT_A_SAT_DNLK_DOWNLINK_OFF, HEXID_METOP_B, SAT_DNLK_OFF);
  check_format_a_downlink(FMT_A_SAT_DNLK_A4_RX_CAPACITY, HEXID_METOP_B, SAT_DNLK_ON_WITH_A4);
  check_format_a_downlink(FMT_A_SAT_DNLK_SPARE_INFO, HEXID_METOP_B, SAT_DNLK_OFF);
  check_format_a_downlink(FMT_A_SAT_DNLK_A3_RX_CAPACITY, HEXID_METOP_B, SAT_DNLK_ON_WITH_A3);
}

TEST(format_a_downlink_forced_off_for_noaa_k_and_noaa_n) {
  // Documented special case: NOAA_K/NOAA_N have no downlink capacity
  // regardless of what the raw status byte says.
  check_format_a_downlink(FMT_A_SAT_DNLK_A3_RX_CAPACITY, HEXID_NOAA_K, SAT_DNLK_OFF);
  check_format_a_downlink(FMT_A_SAT_DNLK_A4_RX_CAPACITY, HEXID_NOAA_N, SAT_DNLK_OFF);
}

static void check_format_a_uplink(enum SatUplinkStatusFormatA_t in, SatHexId_t sat,
                                   enum SatUplinkStatus_t expected) {
  enum SatDownlinkStatus_t dl;
  enum SatUplinkStatus_t ul;
  PREVIPASS_status_format_a_to_generic(FMT_A_SAT_DNLK_DOWNLINK_OFF, in, sat, &dl, &ul);
  CHECK_INT_EQ(ul, expected);
}

TEST(format_a_uplink_table_normal_satellite) {
  check_format_a_uplink(FMT_A_SAT_UPLK_A3_CAPACITY, HEXID_METOP_B, SAT_UPLK_ON_WITH_A3);
  check_format_a_uplink(FMT_A_SAT_UPLK_NEO_CAPACITY, HEXID_METOP_B, SAT_UPLK_ON_WITH_NEO);
  check_format_a_uplink(FMT_A_SAT_UPLK_A4_CAPACITY, HEXID_METOP_B, SAT_UPLK_ON_WITH_A4);
  check_format_a_uplink(FMT_A_SAT_UPLK_OUT_OF_SERVICE, HEXID_METOP_B, SAT_UPLK_OFF);
}

TEST(format_a_uplink_table_noaa_k_and_n_always_collapse_to_a2) {
  // Documented special case: any non-OFF uplink capacity means A2 for
  // NOAA_K/NOAA_N; only OUT_OF_SERVICE stays OFF.
  check_format_a_uplink(FMT_A_SAT_UPLK_A3_CAPACITY, HEXID_NOAA_K, SAT_UPLK_ON_WITH_A2);
  check_format_a_uplink(FMT_A_SAT_UPLK_NEO_CAPACITY, HEXID_NOAA_K, SAT_UPLK_ON_WITH_A2);
  check_format_a_uplink(FMT_A_SAT_UPLK_A4_CAPACITY, HEXID_NOAA_N, SAT_UPLK_ON_WITH_A2);
  check_format_a_uplink(FMT_A_SAT_UPLK_OUT_OF_SERVICE, HEXID_NOAA_N, SAT_UPLK_OFF);
}

// -------------------------------------------------------------------------- //
// Format B -> generic
// -------------------------------------------------------------------------- //

static void check_format_b(enum SatDownlinkStatusFormatB_t dlIn,
                            enum SatOperatingStatusFormatB_t opIn,
                            enum SatDownlinkStatus_t expectedDl,
                            enum SatUplinkStatus_t expectedUl) {
  enum SatDownlinkStatus_t dl;
  enum SatUplinkStatus_t ul;
  PREVIPASS_status_format_b_to_generic(dlIn, opIn, &dl, &ul);
  CHECK_INT_EQ(dl, expectedDl);
  CHECK_INT_EQ(ul, expectedUl);
}

TEST(format_b_downlink_off_forces_generic_downlink_off_regardless_of_operating_status) {
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_OFF, FMT_B_SAT_A3_CAPACITY, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A3);
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_OFF, FMT_B_SAT_A4_CAPACITY, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A4);
}

TEST(format_b_downlink_on_maps_operating_status_to_downlink_capacity) {
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_ON, FMT_B_SAT_A3_CAPACITY, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3);
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_ON, FMT_B_SAT_A4_CAPACITY, SAT_DNLK_ON_WITH_A4, SAT_UPLK_ON_WITH_A4);
  // NEO and OUT_OF_SERVICE have no downlink capacity even with downlink "on".
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_ON, FMT_B_SAT_NEO_CAPACITY, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_NEO);
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_ON, FMT_B_SAT_OUT_OF_SERVICE, SAT_DNLK_OFF, SAT_UPLK_OFF);
}

TEST(format_b_uplink_table_is_independent_of_downlink_flag) {
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_OFF, FMT_B_SAT_NEO_CAPACITY, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_NEO);
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_OFF, FMT_B_SAT_OUT_OF_SERVICE, SAT_DNLK_OFF, SAT_UPLK_OFF);
}

TEST(format_b_uplink_spare_values_default_to_off) {
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_ON, FMT_B_SAT_SPARE_INFO2, SAT_DNLK_OFF, SAT_UPLK_OFF);
  check_format_b(FMT_B_SAT_DNLK_DOWNLINK_ON, FMT_B_SAT_SPARE_INFO5, SAT_DNLK_OFF, SAT_UPLK_OFF);
}

// -------------------------------------------------------------------------- //
// Generic -> format A
// -------------------------------------------------------------------------- //

static void check_generic_to_format_a(enum SatDownlinkStatus_t dlIn, enum SatUplinkStatus_t ulIn,
                                       enum SatDownlinkStatusFormatA_t expectedDl,
                                       enum SatUplinkStatusFormatA_t expectedUl) {
  enum SatDownlinkStatusFormatA_t dl;
  enum SatUplinkStatusFormatA_t ul;
  PREVIPASS_status_generic_to_format_a(dlIn, ulIn, &dl, &ul);
  CHECK_INT_EQ(dl, expectedDl);
  CHECK_INT_EQ(ul, expectedUl);
}

TEST(generic_to_format_a_downlink_table) {
  check_generic_to_format_a(SAT_DNLK_OFF, SAT_UPLK_OFF, FMT_A_SAT_DNLK_DOWNLINK_OFF, FMT_A_SAT_UPLK_OUT_OF_SERVICE);
  check_generic_to_format_a(SAT_DNLK_ON_WITH_A3, SAT_UPLK_OFF, FMT_A_SAT_DNLK_A3_RX_CAPACITY, FMT_A_SAT_UPLK_OUT_OF_SERVICE);
  check_generic_to_format_a(SAT_DNLK_ON_WITH_A4, SAT_UPLK_OFF, FMT_A_SAT_DNLK_A4_RX_CAPACITY, FMT_A_SAT_UPLK_OUT_OF_SERVICE);
}

TEST(generic_to_format_a_uplink_table) {
  check_generic_to_format_a(SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A3, FMT_A_SAT_DNLK_DOWNLINK_OFF, FMT_A_SAT_UPLK_A3_CAPACITY);
  check_generic_to_format_a(SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A4, FMT_A_SAT_DNLK_DOWNLINK_OFF, FMT_A_SAT_UPLK_A4_CAPACITY);
  check_generic_to_format_a(SAT_DNLK_OFF, SAT_UPLK_ON_WITH_NEO, FMT_A_SAT_DNLK_DOWNLINK_OFF, FMT_A_SAT_UPLK_NEO_CAPACITY);
  check_generic_to_format_a(SAT_DNLK_OFF, SAT_UPLK_OFF, FMT_A_SAT_DNLK_DOWNLINK_OFF, FMT_A_SAT_UPLK_OUT_OF_SERVICE);
}

TEST(generic_to_format_a_a2_has_no_format_a_representation_so_falls_to_out_of_service) {
  // Format A has no slot for "Argos 2" uplink -- SAT_UPLK_ON_WITH_A2 hits
  // the `default` branch. Documents current (lossy) behavior.
  check_generic_to_format_a(SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A2, FMT_A_SAT_DNLK_DOWNLINK_OFF, FMT_A_SAT_UPLK_OUT_OF_SERVICE);
}

int main(void) {
  RUN_TEST(format_a_downlink_table);
  RUN_TEST(format_a_downlink_forced_off_for_noaa_k_and_noaa_n);
  RUN_TEST(format_a_uplink_table_normal_satellite);
  RUN_TEST(format_a_uplink_table_noaa_k_and_n_always_collapse_to_a2);
  RUN_TEST(format_b_downlink_off_forces_generic_downlink_off_regardless_of_operating_status);
  RUN_TEST(format_b_downlink_on_maps_operating_status_to_downlink_capacity);
  RUN_TEST(format_b_uplink_table_is_independent_of_downlink_flag);
  RUN_TEST(format_b_uplink_spare_values_default_to_off);
  RUN_TEST(generic_to_format_a_downlink_table);
  RUN_TEST(generic_to_format_a_uplink_table);
  RUN_TEST(generic_to_format_a_a2_has_no_format_a_representation_so_falls_to_out_of_service);
  TEST_REPORT_AND_EXIT();
}
