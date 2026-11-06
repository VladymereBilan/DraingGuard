// Proves include/config.h is self-contained (decision D24).
//
// config.h is included first, before unity.h and before anything else, in a host
// build where Arduino.h does not exist and secrets.h is not on the include path.
// If config.h ever grows a dependency on either, this test stops compiling.

#include "config.h"

#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_config_is_self_contained(void) {
  TEST_ASSERT_EQUAL_UINT8(25, cfg::MOTOR_IN1);
  TEST_ASSERT_EQUAL_INT(10, cfg::SENSOR_STALE_SAMPLES);
  TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0343f, cfg::SOUND_CM_PER_US);
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_config_is_self_contained);
  return UNITY_END();
}
