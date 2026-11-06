// Unity tests for the dg_core sensor filter pipeline. Runs on the host:
//   pio test -e native
//
// The filter is configured from include/config.h so the tests exercise the real
// tunables rather than a parallel set of numbers.

#include <unity.h>

#include "config.h"
#include "dg_filters.h"

void setUp(void) {}
void tearDown(void) {}

namespace {

constexpr dg::FilterParams kParams = {cfg::RANGE_MIN_CM, cfg::RANGE_MAX_CM,
                                      cfg::SENSOR_STALE_SAMPLES};

// Pushes n copies of cm into f.
void pushN(dg::DistanceFilter& f, float cm, int n) {
  for (int i = 0; i < n; ++i) {
    f.push(cm);
  }
}

// Pushes enough identical samples that the moving average is full.
void warmUp(dg::DistanceFilter& f, float cm) {
  pushN(f, cm, 12);
}

} // namespace

// 1. Echo timing conversion.
static void test_echoUsToCm(void) {
  // 1749 us round trip at 0.0343 cm/us is ~30 cm one way.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f,
                           dg::echoUsToCm(1749u, cfg::SOUND_CM_PER_US));
  // 0 us means the echo never returned.
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -1.0f,
                           dg::echoUsToCm(0u, cfg::SOUND_CM_PER_US));
}

// 2. Level from a mount height and a measured distance.
static void test_levelFromDistance(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 10.0f, dg::levelFromDistance(40.0f, 30.0f));
}

// 3. medianOf: odd n, even n, n == 1, and the input is left alone.
static void test_medianOf(void) {
  const float odd5[5] = {5.0f, 1.0f, 4.0f, 2.0f, 3.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, dg::medianOf(odd5, 5));

  const float odd3[3] = {5.0f, 1.0f, 3.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, dg::medianOf(odd3, 3));

  // Even n: mean of the two middle values.
  const float even4[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.5f, dg::medianOf(even4, 4));

  const float even2[2] = {10.0f, 20.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 15.0f, dg::medianOf(even2, 2));

  const float one[1] = {7.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 7.0f, dg::medianOf(one, 1));

  // Out-of-contract n is rejected rather than read out of bounds.
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -1.0f, dg::medianOf(odd5, 0));
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -1.0f, dg::medianOf(odd5, 6));

  // The caller's array must come back untouched.
  float input[5] = {40.0f, 2.0f, 38.0f, 200.0f, 39.0f};
  const float expected[5] = {40.0f, 2.0f, 38.0f, 200.0f, 39.0f};
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 39.0f, dg::medianOf(input, 5));
  for (int i = 0; i < 5; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected[i], input[i]);
  }
}

// 4. A steady stream settles on its own value and reports ready.
static void test_constant_stream_becomes_ready(void) {
  dg::DistanceFilter f(kParams);

  // Three pushes fill the median window, then four more fill the average.
  pushN(f, 30.0f, dg::DistanceFilter::kMinValidInWindow +
                      dg::DistanceFilter::kAvgWindow - 2);
  TEST_ASSERT_FALSE(f.ready());

  f.push(30.0f);
  TEST_ASSERT_TRUE(f.ready());
  TEST_ASSERT_TRUE(f.valid());
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());
}

// 5. A single spike never reaches the output.
static void test_single_spike_is_rejected(void) {
  dg::DistanceFilter f(kParams);
  const float stream[8] = {30.0f, 30.0f, 150.0f, 30.0f,
                           30.0f, 30.0f,  30.0f, 30.0f};

  for (int i = 0; i < 8; ++i) {
    f.push(stream[i]);
    if (f.valid()) {
      TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, f.value());
    }
  }
  TEST_ASSERT_TRUE(f.ready());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, f.value());
}

// 6. Samples outside [minCm, maxCm] are ignored, not blended in.
static void test_out_of_range_samples_ignored(void) {
  dg::DistanceFilter f(kParams);
  warmUp(f, 30.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());

  f.push(1.0f); // below RANGE_MIN_CM
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());

  f.push(250.0f); // above RANGE_MAX_CM
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());

  f.push(-1.0f); // echo timeout sentinel
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());

  TEST_ASSERT_TRUE(f.valid());
}

// 7. A real step is tracked without overshoot.
static void test_step_response(void) {
  dg::DistanceFilter f(kParams);
  warmUp(f, 30.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());

  // Eight pushes is enough for the median window and the average to turn over.
  for (int i = 0; i < 8; ++i) {
    f.push(20.0f);
    // Monotone approach from above: never dips under the new level.
    TEST_ASSERT_TRUE(f.value() >= 20.0f - 0.0001f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, f.value());
}

// 8. A total dropout eventually marks the sensor invalid (D25).
static void test_goes_invalid_when_stale(void) {
  dg::DistanceFilter f(kParams);
  warmUp(f, 30.0f);
  TEST_ASSERT_TRUE(f.valid());

  // The first pushes after good data still leave kMinValidInWindow valid
  // samples in the window, so they keep producing medians.
  const int stillProducing =
      dg::DistanceFilter::kRawWindow - dg::DistanceFilter::kMinValidInWindow;
  for (int i = 0; i < stillProducing; ++i) {
    f.push(-1.0f);
    TEST_ASSERT_TRUE(f.valid());
  }

  // From here every push produces no median. The sensor stays valid for
  // staleSamples - 1 of them and flips on the staleSamples-th.
  for (int i = 1; i < cfg::SENSOR_STALE_SAMPLES; ++i) {
    f.push(-1.0f);
    TEST_ASSERT_TRUE(f.valid());
  }
  f.push(-1.0f);
  TEST_ASSERT_FALSE(f.valid());
}

// 9. Good data after a dropout brings the sensor back.
static void test_recovers_after_stale(void) {
  dg::DistanceFilter f(kParams);
  warmUp(f, 30.0f);

  pushN(f, -1.0f, dg::DistanceFilter::kRawWindow + cfg::SENSOR_STALE_SAMPLES);
  TEST_ASSERT_FALSE(f.valid());

  // A median needs kMinValidInWindow good samples back in the window.
  for (int i = 1; i < dg::DistanceFilter::kMinValidInWindow; ++i) {
    f.push(25.0f);
    TEST_ASSERT_FALSE(f.valid());
  }
  f.push(25.0f);
  TEST_ASSERT_TRUE(f.valid());
}

// 10. Fewer than kMinValidInWindow valid samples produces no median at all.
static void test_too_few_valid_produces_no_median(void) {
  dg::DistanceFilter f(kParams);

  // Two valid samples are not enough: no median, so no value yet.
  f.push(30.0f);
  TEST_ASSERT_FALSE(f.valid());
  f.push(30.0f);
  TEST_ASSERT_FALSE(f.valid());
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, f.value());

  // The third completes the quorum.
  f.push(30.0f);
  TEST_ASSERT_TRUE(f.valid());
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());

  // Once the window drops back below quorum the value freezes instead of drifting.
  pushN(f, -1.0f, dg::DistanceFilter::kRawWindow);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 30.0f, f.value());
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_echoUsToCm);
  RUN_TEST(test_levelFromDistance);
  RUN_TEST(test_medianOf);
  RUN_TEST(test_constant_stream_becomes_ready);
  RUN_TEST(test_single_spike_is_rejected);
  RUN_TEST(test_out_of_range_samples_ignored);
  RUN_TEST(test_step_response);
  RUN_TEST(test_goes_invalid_when_stale);
  RUN_TEST(test_recovers_after_stale);
  RUN_TEST(test_too_few_valid_produces_no_median);
  return UNITY_END();
}
