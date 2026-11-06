// S4a test suite: the logger's pure logic - slot codec, ring with persisted
// acknowledgement, timestamp backfill, request bodies and the pending queue.
// Run with:  pio test -e native

#include <unity.h>

#include "tests_logger.h"

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();

  // Slot codec
  RUN_TEST(dgtest::test_l1_slot_round_trip);
  RUN_TEST(dgtest::test_l2_flipped_bit_fails_crc);

  // Ring
  RUN_TEST(dgtest::test_l3_mount_blank_store);
  RUN_TEST(dgtest::test_l4_append_then_oldest_first);
  RUN_TEST(dgtest::test_l5_overwrite_counts_as_dropped);
  RUN_TEST(dgtest::test_l6_mount_rebuilds_after_reboot);
  RUN_TEST(dgtest::test_l7_mount_across_slot_wrap);
  RUN_TEST(dgtest::test_l8_mount_skips_torn_slot);
  RUN_TEST(dgtest::test_l9_drop_oldest_advances_tail_only);

  // Timestamps
  RUN_TEST(dgtest::test_l10_backfill_current_boot_synced);
  RUN_TEST(dgtest::test_l11_wait_when_not_synced);
  RUN_TEST(dgtest::test_l12_drop_old_boot_undated);
  RUN_TEST(dgtest::test_l13_old_boot_already_dated);
  RUN_TEST(dgtest::test_l14_backfill_across_millis_rollover);
  RUN_TEST(dgtest::test_l15_iso8601_known_epochs);

  // Request bodies
  RUN_TEST(dgtest::test_l16_bulk_body_entry_counts);
  RUN_TEST(dgtest::test_l17_bulk_body_overflow_stays_valid);
  RUN_TEST(dgtest::test_l18_duplicate_timestamps_bumped);
  RUN_TEST(dgtest::test_l19_update_body_fields_and_encoding);

  // Pending queue
  RUN_TEST(dgtest::test_l20_snapshot_replaces_adjacent_snapshot);
  RUN_TEST(dgtest::test_l21_snapshot_after_event_appends);
  RUN_TEST(dgtest::test_l22_events_keep_order);
  RUN_TEST(dgtest::test_l23_full_queue_refuses_push);

  // Accounting
  RUN_TEST(dgtest::test_l24_dropped_accounting);

  // Persisted acknowledgement (D53)
  RUN_TEST(dgtest::test_l25_ack_not_resurfaced_after_reboot);
  RUN_TEST(dgtest::test_l26_partial_ack_leaves_only_unacked);

  // Sensor validity (D62)
  RUN_TEST(dgtest::test_l27_codec_round_trips_validity_flags);
  RUN_TEST(dgtest::test_l28_update_body_omits_invalid_fields);
  RUN_TEST(dgtest::test_l29_bulk_body_omits_invalid_fields);

  // Routing (D65)
  RUN_TEST(dgtest::test_l30_offline_records_always_ring);
  RUN_TEST(dgtest::test_l31_coalescing_only_online_in_window);
  RUN_TEST(dgtest::test_l32_outage_then_single_ordered_flush);
  RUN_TEST(dgtest::test_l33_pending_push_respects_permission);
  RUN_TEST(dgtest::test_l34_drop_accounting_covers_every_exit);

  // Upload responses (D67)
  RUN_TEST(dgtest::test_l35_bulk_202_is_success);
  RUN_TEST(dgtest::test_l36_update_needs_nonzero_id);

  // Flash inhibit while the motor runs (D66)
  RUN_TEST(dgtest::test_l37_flash_inhibited_never_rings);
  RUN_TEST(dgtest::test_l38_pending_capacity_covers_longest_stroke);
  RUN_TEST(dgtest::test_l39_pending_drains_to_ring_in_order);

  return UNITY_END();
}
