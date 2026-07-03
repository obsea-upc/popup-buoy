/*
 * Native unit tests for the pass-prediction entry points in previpass.c:
 * PREVIPASS_compute_next_pass[_with_status] and
 * PREVIPASS_compute_new_prediction_pass_times[_with_status].
 *
 * Orbital parameters below are a plausible sun-synchronous LEO satellite
 * (~7226 km semi-major axis, ~99.2 deg inclination, ~102 min period --
 * broadly NOAA/METOP-class), picked to be internally consistent and to
 * produce a deterministic, verified-by-hand pass over Barcelona
 * (41.2N, 2.1E) in the chosen prediction window. They are not a real
 * orbital bulletin.
 */
#include "framework.h"
#include "previpass.h"

static struct AopSatelliteEntry_t make_test_satellite(void) {
  struct AopSatelliteEntry_t sat = PREVIPASS_default_aop_satellite_entry();
  sat.satHexId = HEXID_NOAA_N;
  sat.downlinkStatus = SAT_DNLK_OFF;
  sat.uplinkStatus = SAT_UPLK_ON_WITH_A3;
  sat.bulletin = (struct CalendarDateTime_t){2024, 1, 1, 0, 0, 0};
  sat.semiMajorAxisKm = 7226.13f;
  sat.inclinationDeg = 99.19f;
  sat.ascNodeLongitudeDeg = 100.0f;
  sat.ascNodeDriftDeg = -25.6f;
  sat.orbitPeriodMin = 102.12f;
  sat.semiMajorAxisDriftMeterPerDay = 0.0f;
  return sat;
}

static struct PredictionPassConfiguration_t make_test_config(float minElevation,
                                                               float maxElevation) {
  struct PredictionPassConfiguration_t cfg = {
      41.2f,                            // gpsLat (Barcelona-ish)
      2.1f,                             // gpsLong
      {2024, 1, 2, 0, 0, 0},            // start
      {2024, 1, 3, 0, 0, 0},            // end (24h window)
      minElevation,
      maxElevation,
      1.0f,                             // minPassDurationMinute
      1000,                             // maxPasses
      0.0f,                             // timeMarginMinPer6months
      30,                               // computationStepSecond
  };
  return cfg;
}

TEST(compute_next_pass_finds_a_pass_within_24h_at_moderate_elevation) {
  struct AopSatelliteEntry_t sat = make_test_satellite();
  struct PredictionPassConfiguration_t cfg = make_test_config(5.0f, 90.0f);

  struct SatelliteNextPassPrediction_t nextPass;
  bool found = PREVIPASS_compute_next_pass(&cfg, &sat, 1, &nextPass);

  CHECK(found);
  CHECK(nextPass.duration > 0);
  CHECK(nextPass.elevationMax >= 5);
  CHECK_INT_EQ(nextPass.satHexId, HEXID_NOAA_N);
}

TEST(compute_next_pass_returns_false_when_elevation_window_is_unreachable) {
  struct AopSatelliteEntry_t sat = make_test_satellite();
  // A 0.1-degree-wide elevation band this satellite's geometry never lands in.
  struct PredictionPassConfiguration_t cfg = make_test_config(89.9f, 90.0f);

  struct SatelliteNextPassPrediction_t nextPass;
  bool found = PREVIPASS_compute_next_pass(&cfg, &sat, 1, &nextPass);

  CHECK(!found);
}

TEST(compute_next_pass_with_status_excludes_satellites_below_requested_capacity) {
  struct AopSatelliteEntry_t sat = make_test_satellite();  // uplink = A3 only
  struct PredictionPassConfiguration_t cfg = make_test_config(5.0f, 90.0f);

  struct SatelliteNextPassPrediction_t nextPass;
  // Requesting NEO-or-better uplink excludes an A3-only satellite.
  bool found = PREVIPASS_compute_next_pass_with_status(&cfg, &sat, 1, SAT_DNLK_OFF,
                                                         SAT_UPLK_ON_WITH_NEO, &nextPass);
  CHECK(!found);

  // Requesting A3-or-better (i.e. exactly what the satellite has) succeeds.
  bool foundOk = PREVIPASS_compute_next_pass_with_status(&cfg, &sat, 1, SAT_DNLK_OFF,
                                                           SAT_UPLK_ON_WITH_A3, &nextPass);
  CHECK(foundOk);
}

TEST(compute_new_prediction_pass_times_sets_overflow_when_pool_is_exhausted) {
  // MY_MALLOC_MAX_BYTES=20 and each list element is 16 bytes, so a config
  // that yields 2+ passes over a full day at minElevation=0 (~14
  // passes/day for this orbit) must overflow the pool on the 2nd insert.
  struct AopSatelliteEntry_t sat = make_test_satellite();
  struct PredictionPassConfiguration_t cfg = make_test_config(0.0f, 90.0f);

  bool overflow = false;
  struct SatPassLinkedListElement_t *list =
      PREVIPASS_compute_new_prediction_pass_times(&cfg, &sat, 1, &overflow);

  CHECK(overflow);
  CHECK(list != NULL);  // at least the one pass that fit is still returned
}

TEST(compute_new_prediction_pass_times_no_overflow_for_narrow_window) {
  struct AopSatelliteEntry_t sat = make_test_satellite();
  struct PredictionPassConfiguration_t cfg = make_test_config(0.0f, 90.0f);
  cfg.end = (struct CalendarDateTime_t){2024, 1, 2, 2, 0, 0};  // 2h window

  bool overflow = false;
  PREVIPASS_compute_new_prediction_pass_times(&cfg, &sat, 1, &overflow);

  CHECK(!overflow);
}

TEST(default_aop_satellite_entry_has_all_capacities_off) {
  struct AopSatelliteEntry_t sat = PREVIPASS_default_aop_satellite_entry();
  CHECK_INT_EQ(sat.satHexId, HEXID_INVALID);
  CHECK_INT_EQ(sat.downlinkStatus, SAT_DNLK_OFF);
  CHECK_INT_EQ(sat.uplinkStatus, SAT_UPLK_OFF);
}

int main(void) {
  RUN_TEST(compute_next_pass_finds_a_pass_within_24h_at_moderate_elevation);
  RUN_TEST(compute_next_pass_returns_false_when_elevation_window_is_unreachable);
  RUN_TEST(compute_next_pass_with_status_excludes_satellites_below_requested_capacity);
  RUN_TEST(compute_new_prediction_pass_times_sets_overflow_when_pool_is_exhausted);
  RUN_TEST(compute_new_prediction_pass_times_no_overflow_for_narrow_window);
  RUN_TEST(default_aop_satellite_entry_has_all_capacities_off);
  TEST_REPORT_AND_EXIT();
}
