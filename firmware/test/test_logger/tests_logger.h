#ifndef DG_TESTS_LOGGER_H
#define DG_TESTS_LOGGER_H

namespace dgtest {

// Slot codec
void test_l1_slot_round_trip();
void test_l2_flipped_bit_fails_crc();

// Ring
void test_l3_mount_blank_store();
void test_l4_append_then_oldest_first();
void test_l5_overwrite_counts_as_dropped();
void test_l6_mount_rebuilds_after_reboot();
void test_l7_mount_across_slot_wrap();
void test_l8_mount_skips_torn_slot();
void test_l9_drop_oldest_advances_tail_only();

// Timestamps
void test_l10_backfill_current_boot_synced();
void test_l11_wait_when_not_synced();
void test_l12_drop_old_boot_undated();
void test_l13_old_boot_already_dated();
void test_l14_backfill_across_millis_rollover();
void test_l15_iso8601_known_epochs();

// Bulk / update bodies
void test_l16_bulk_body_entry_counts();
void test_l17_bulk_body_overflow_stays_valid();
void test_l18_duplicate_timestamps_bumped();
void test_l19_update_body_fields_and_encoding();

// Pending queue
void test_l20_snapshot_replaces_adjacent_snapshot();
void test_l21_snapshot_after_event_appends();
void test_l22_events_keep_order();
void test_l23_full_queue_refuses_push();

// Accounting
void test_l24_dropped_accounting();

// Persisted acknowledgement (D53)
void test_l25_ack_not_resurfaced_after_reboot();
void test_l26_partial_ack_leaves_only_unacked();

// Sensor validity never uploaded as data (D62)
void test_l27_codec_round_trips_validity_flags();
void test_l28_update_body_omits_invalid_fields();
void test_l29_bulk_body_omits_invalid_fields();

} // namespace dgtest

#endif // DG_TESTS_LOGGER_H
