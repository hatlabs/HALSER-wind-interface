// Host-side unit tests for the vane reference-angle range bound (pio test -e
// native). The bound is pure and hardware-independent; the ArduinoJson type gate
// in AutonnicReferenceAngleConfig::from_json needs the device runtime and is not
// covered here.

#include <unity.h>

#include <cmath>

#include "../../src/reference_angle.h"

using namespace wind_interface;

namespace {
// 180/pi, matching the schema's displayMultiplier: the web UI stores an entered
// degrees value as degrees / kDegPerRad = radians.
constexpr double kDegPerRad = 57.29577951308232;
}  // namespace

void test_accepts_zero() { TEST_ASSERT_TRUE(reference_angle_in_range(0.0)); }

// An exact +/-180 deg entry, converted deg->rad the way the web UI does, must
// survive the float round-trip and be accepted (the reason for the slack).
void test_accepts_plus_180() {
  TEST_ASSERT_TRUE(reference_angle_in_range(180.0 / kDegPerRad));
}

void test_accepts_minus_180() {
  TEST_ASSERT_TRUE(reference_angle_in_range(-180.0 / kDegPerRad));
}

void test_accepts_exact_pi() {
  TEST_ASSERT_TRUE(reference_angle_in_range(M_PI));
  TEST_ASSERT_TRUE(reference_angle_in_range(-M_PI));
}

// Just inside the slack is accepted; clearly past it is rejected.
void test_slack_boundary() {
  TEST_ASSERT_TRUE(reference_angle_in_range(M_PI + kReferenceAngleSlackRad / 2));
  TEST_ASSERT_FALSE(reference_angle_in_range(M_PI + 1e-3));
}

void test_rejects_just_over_180() {
  TEST_ASSERT_FALSE(reference_angle_in_range(181.0 / kDegPerRad));
}

void test_rejects_non_finite() {
  TEST_ASSERT_FALSE(reference_angle_in_range(NAN));
  TEST_ASSERT_FALSE(reference_angle_in_range(INFINITY));
  TEST_ASSERT_FALSE(reference_angle_in_range(-INFINITY));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_accepts_zero);
  RUN_TEST(test_accepts_plus_180);
  RUN_TEST(test_accepts_minus_180);
  RUN_TEST(test_accepts_exact_pi);
  RUN_TEST(test_slack_boundary);
  RUN_TEST(test_rejects_just_over_180);
  RUN_TEST(test_rejects_non_finite);
  return UNITY_END();
}
