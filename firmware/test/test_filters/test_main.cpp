// Unity tests for dg::median5. Runs on the native environment:
//   pio test -e native

#include <unity.h>

#include "dg_filters.h"

void setUp(void) {}
void tearDown(void) {}

static void test_median5_sorted(void) {
  const float v[5] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, dg::median5(v));
}

static void test_median5_reverse_sorted(void) {
  const float v[5] = {5.0f, 4.0f, 3.0f, 2.0f, 1.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, dg::median5(v));
}

static void test_median5_duplicates(void) {
  const float v[5] = {7.0f, 7.0f, 7.0f, 1.0f, 9.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 7.0f, dg::median5(v));
}

static void test_median5_negatives(void) {
  const float v[5] = {-5.0f, -1.0f, -3.0f, -2.0f, -4.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -3.0f, dg::median5(v));
}

static void test_median5_leaves_input_unchanged(void) {
  float v[5] = {40.0f, 2.0f, 38.0f, 200.0f, 39.0f};
  const float expected[5] = {40.0f, 2.0f, 38.0f, 200.0f, 39.0f};

  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 39.0f, dg::median5(v));

  for (int i = 0; i < 5; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected[i], v[i]);
  }
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_median5_sorted);
  RUN_TEST(test_median5_reverse_sorted);
  RUN_TEST(test_median5_duplicates);
  RUN_TEST(test_median5_negatives);
  RUN_TEST(test_median5_leaves_input_unchanged);
  return UNITY_END();
}
