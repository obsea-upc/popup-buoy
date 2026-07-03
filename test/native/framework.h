/*
 * Tiny header-only assert-and-report test framework for the native (host
 * machine) unit tests under test/native/. No external dependency (no
 * PlatformIO/Unity/GoogleTest download required) -- just gcc/g++ and libm.
 *
 * Each test .c file is its own standalone executable with its own main();
 * run_tests.sh builds and runs all of them and aggregates the result.
 */
#ifndef POPUP_BUOY_TEST_FRAMEWORK_H
#define POPUP_BUOY_TEST_FRAMEWORK_H

#include <math.h>
#include <stdio.h>

static int g_tests_run = 0;
static int g_tests_failed = 0;
static const char *g_current_test = "";

#define TEST(name) static void name(void)

#define RUN_TEST(name)                     \
  do {                                     \
    g_current_test = #name;                \
    g_tests_run++;                         \
    name();                                \
  } while (0)

#define CHECK(cond)                                                         \
  do {                                                                      \
    if (!(cond)) {                                                          \
      fprintf(stderr, "FAIL %s:%d [%s] %s\n", __FILE__, __LINE__,           \
              g_current_test, #cond);                                       \
      g_tests_failed++;                                                     \
    }                                                                       \
  } while (0)

#define CHECK_INT_EQ(a, b)                                                  \
  do {                                                                      \
    long long _a = (long long)(a), _b = (long long)(b);                    \
    if (_a != _b) {                                                         \
      fprintf(stderr, "FAIL %s:%d [%s] %s (%lld) != %s (%lld)\n", __FILE__, \
              __LINE__, g_current_test, #a, _a, #b, _b);                    \
      g_tests_failed++;                                                     \
    }                                                                       \
  } while (0)

#define CHECK_STR_EQ(a, b)                                                  \
  do {                                                                      \
    const char *_a = (a), *_b = (b);                                        \
    if ((_a == NULL) != (_b == NULL) || (_a && _b && strcmp(_a, _b) != 0)) { \
      fprintf(stderr, "FAIL %s:%d [%s] %s (\"%s\") != %s (\"%s\")\n",       \
              __FILE__, __LINE__, g_current_test, #a, _a ? _a : "(null)",   \
              #b, _b ? _b : "(null)");                                      \
      g_tests_failed++;                                                     \
    }                                                                       \
  } while (0)

#define CHECK_FLOAT_NEAR(a, b, eps)                                         \
  do {                                                                      \
    double _a = (double)(a), _b = (double)(b), _eps = (double)(eps);        \
    if (fabs(_a - _b) > _eps) {                                             \
      fprintf(stderr, "FAIL %s:%d [%s] %s (%f) != %s (%f) within %f\n",     \
              __FILE__, __LINE__, g_current_test, #a, _a, #b, _b, _eps);    \
      g_tests_failed++;                                                     \
    }                                                                       \
  } while (0)

#define TEST_REPORT_AND_EXIT()                                              \
  do {                                                                      \
    fprintf(stderr, "\n%d test(s) run, %d failure(s) -- %s\n", g_tests_run, \
            g_tests_failed, g_tests_failed ? "FAIL" : "OK");                \
    return g_tests_failed ? 1 : 0;                                          \
  } while (0)

#endif  // POPUP_BUOY_TEST_FRAMEWORK_H
