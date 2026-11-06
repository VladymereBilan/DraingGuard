// The logger: Wi-Fi, NTP, ThingSpeak uploads and the offline ring, all inside
// one FreeRTOS task pinned to core 0 (D29).
//
// The control loop on core 1 only ever calls submit(), which is a
// xQueueSend(..., 0) and therefore cannot block or fail slowly. Everything
// that waits - association, DNS, TCP, HTTP, NTP - happens in the task. A queue
// that is full counts the record as dropped rather than stalling the loop.
//
// Ordering: while the ring holds anything, every new record goes through the
// ring, so the channel stays chronological with no merge logic (D51).
//
// Secrets: the write key is held only to put it in a request body. It is never
// printed, and no "#NET" line contains it (D58).

#ifndef DG_NET_LOGGER_H
#define DG_NET_LOGGER_H

#include <atomic>
#include <stdint.h>

#include "boot_id.h"
#include "dg_bulk_json.h"
#include "dg_controller.h"
#include "dg_pending.h"
#include "dg_route.h"
#include "dg_upload_result.h"
#include "dg_record_codec.h"
#include "dg_ring.h"
#include "fs_slots.h"

namespace hal {

enum class NetState : uint8_t {
  FS_MOUNT,
  DISCONNECTED,
  CONNECTING,
  SYNCING_TIME,
  ONLINE,
  FLUSHING,
};

struct LoggerStats {
  uint32_t uploadsOk;
  uint32_t uploadsFailed;
  uint32_t bulkFlushes;
  uint32_t droppedQueueFull;      // written by the control loop
  uint32_t droppedRingOverwrite;  // written by the task
  uint32_t droppedOldBoot;        // written by the task
  uint32_t droppedNoRing;         // FS unavailable, nowhere to buffer
  uint32_t droppedCoalesced;      // snapshot superseded before upload (D65)
  uint32_t droppedFlashInhibited; // RAM full while the motor ran (D66)
  uint16_t ringCount;
  uint32_t freeHeap;
  uint32_t minFreeHeap;
  uint32_t stackHighWater; // bytes of task stack never used
  bool wifiUp;
  bool timeSynced;
  bool fsReady;
  bool fsFormatted;
  NetState state;
};

class NetLogger {
public:
  NetLogger();

  // ssid, pass, channelId and writeKey must outlive the logger. Starts the
  // task; returns false only if the queue or task could not be created.
  bool begin(const char* ssid, const char* pass, const char* channelId,
             const char* writeKey, const NetPersist& persisted);

  // Called from the 200 ms control tick. Never blocks.
  void submit(const dg::Record& r, uint32_t uptimeMs);

  // While the pointed-to flag reads true the logger performs no flash writes
  // at all - no ring append, no ack, no NVS - because a write stalls the
  // instruction cache and with it the 10 ms safety loop (D66). The caller owns
  // the flag; nullptr means never inhibited.
  void setMotionActiveFlag(const std::atomic<bool>* flag) {
    motionActive_ = flag;
  }

  // ThingSpeak field 8: queue overruns + ring overwrites + undatable records.
  uint32_t droppedTotal() const;

  const LoggerStats& stats() const { return stats_; }

#ifdef DG_NET_DEBUG
  // Simulates losing Wi-Fi, so the offline path can be exercised in Wokwi.
  void setNetEnabled(bool on);
  bool netEnabled() const { return netEnabled_; }
#endif

private:
  static void taskTrampoline(void* arg);
  void taskLoop();

  dg::LinkState linkState() const;
  bool flashInhibited() const;
  void drainQueue();
  void agePending();
  void pendingToRing(const char* why);
  void appendToRing(const dg::StoredRecord& r);
  bool uploadOneLive();
  bool flushRing();
  void persistCounters();

  bool goOffline(const char* why);
  uint32_t nowEpoch() const;
  bool spacingElapsed() const;

  bool postBody(const char* path, const char* contentType, const char* body,
                uint16_t len, char* resp, uint16_t respCap, int& httpCode);

  const char* ssid_;
  const char* pass_;
  const char* channelId_;
  const char* writeKey_;

  uint16_t bootId_;
  uint32_t droppedBase_; // carried over from NVS (D48)
  uint32_t lastAckSeq_;  // carried over from NVS (D53)

  NetState state_;
  bool timeSynced_;
  bool netEnabled_;

  const std::atomic<bool>* motionActive_;
  bool wasInhibited_;
  bool deferredWrites_;

  uint32_t stateSinceMs_;
  uint32_t lastAttemptMs_;
  uint32_t backoffMs_;
  uint32_t lastUploadMs_;
  uint32_t lastNtpMs_;
  uint32_t oldestPendingMs_;

  FsSlots slots_;
  dg::Ring ring_;
  dg::PendingQueue pending_;
  NetStore store_;

  void* queue_; // QueueHandle_t, kept opaque to spare callers the FreeRTOS headers
  void* task_;  // TaskHandle_t

  LoggerStats stats_;
};

} // namespace hal

#endif // DG_NET_LOGGER_H
