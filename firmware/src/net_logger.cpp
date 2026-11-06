#include "net_logger.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "config.h"
#include "dg_timestamp.h"

namespace hal {
namespace {

// Network policy lives here, not in config.h: none of it is a control tunable
// and none of it affects the state machine (D55).
constexpr uint32_t kTaskStackBytes = 6144;
constexpr UBaseType_t kTaskPriority = 1;
constexpr BaseType_t kTaskCore = 0;
constexpr uint32_t kTaskTickMs = 100;

constexpr uint32_t kBackoffMinMs = 1000;
constexpr uint32_t kBackoffMaxMs = 30000;
constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kSyncTimeoutMs = 30000;
constexpr uint32_t kNtpResyncMs = 3600000; // once an hour
constexpr uint16_t kHttpTimeoutMs = 8000;

// Records wait this long in RAM for NTP before being written to the ring,
// where they can still be dated later in the same boot (D52).
constexpr uint32_t kPendingHoldMs = 60000;

// An epoch below this means the clock has not actually been set.
constexpr uint32_t kEpochSane = 1700000000u; // 2023-11-14

constexpr char kHost[] = "api.thingspeak.com";
constexpr uint16_t kPort = 80;

// 60 entries measure 8.3 KB typical and 9.5 KB worst case, so 10 KB (D50).
constexpr uint16_t kBulkBufBytes = 10240;
constexpr uint16_t kMaxBulkEntries = 60;

char g_bulkBuf[kBulkBufBytes];
char g_formBuf[256];
char g_respBuf[64];
char g_pathBuf[64];

} // namespace

NetLogger::NetLogger()
    : ssid_(nullptr),
      pass_(nullptr),
      channelId_(nullptr),
      writeKey_(nullptr),
      bootId_(0),
      droppedBase_(0),
      lastAckSeq_(0),
      state_(NetState::FS_MOUNT),
      timeSynced_(false),
      netEnabled_(true),
      stateSinceMs_(0),
      lastAttemptMs_(0),
      backoffMs_(kBackoffMinMs),
      lastUploadMs_(0),
      lastNtpMs_(0),
      oldestPendingMs_(0),
      slots_(),
      ring_(slots_),
      pending_(),
      store_(),
      queue_(nullptr),
      task_(nullptr),
      stats_{} {}

uint32_t NetLogger::droppedTotal() const {
  // Each counter has exactly one writer and 32-bit aligned loads are atomic on
  // this part, so summing them without a lock can be momentarily stale but
  // never torn.
  return droppedBase_ + stats_.droppedQueueFull + stats_.droppedRingOverwrite +
         stats_.droppedOldBoot + stats_.droppedNoRing +
         stats_.droppedCoalesced;
}

bool NetLogger::begin(const char* ssid, const char* pass,
                      const char* channelId, const char* writeKey,
                      const NetPersist& persisted) {
  ssid_ = ssid;
  pass_ = pass;
  channelId_ = channelId;
  writeKey_ = writeKey;
  bootId_ = persisted.bootId;
  droppedBase_ = persisted.droppedTotal;
  lastAckSeq_ = persisted.lastAckSeq;
  // The caller already did the one beginBoot() for this boot; just tell our
  // own store what is on flash so the first save is not a rewrite.
  store_.seed(persisted.lastAckSeq, persisted.droppedTotal);

  queue_ = xQueueCreate(cfg::LOG_QUEUE_DEPTH, sizeof(dg::StoredRecord));
  if (queue_ == nullptr) {
    return false;
  }

  TaskHandle_t handle = nullptr;
  const BaseType_t ok = xTaskCreatePinnedToCore(
      &NetLogger::taskTrampoline, "dg_net", kTaskStackBytes, this,
      kTaskPriority, &handle, kTaskCore);
  if (ok != pdPASS) {
    return false;
  }
  task_ = handle;
  return true;
}

void NetLogger::submit(const dg::Record& r, uint32_t uptimeMs) {
  dg::StoredRecord sr{};
  sr.seq = 0; // assigned if and when it reaches the ring
  sr.uptimeMs = uptimeMs;
  sr.epoch = 0; // dated later (D8)
  sr.bootId = bootId_;
  sr.rec = r;

  if (queue_ == nullptr) {
    ++stats_.droppedQueueFull;
    return;
  }
  // Zero ticks: the control loop never waits on the logger.
  if (xQueueSend(static_cast<QueueHandle_t>(queue_), &sr, 0) != pdTRUE) {
    ++stats_.droppedQueueFull;
  }
}

#ifdef DG_NET_DEBUG
void NetLogger::setNetEnabled(bool on) { netEnabled_ = on; }
#endif

void NetLogger::taskTrampoline(void* arg) {
  static_cast<NetLogger*>(arg)->taskLoop();
}

uint32_t NetLogger::nowEpoch() const {
  const time_t t = time(nullptr);
  if (t < static_cast<time_t>(kEpochSane)) {
    return 0;
  }
  return static_cast<uint32_t>(t);
}

bool NetLogger::spacingElapsed() const {
  return (millis() - lastUploadMs_) >= cfg::UPLOAD_SPACING_MS;
}

bool NetLogger::goOffline(const char* why) {
  WiFi.disconnect(true);
  state_ = NetState::DISCONNECTED;
  stateSinceMs_ = millis();
  stats_.wifiUp = false;
  Serial.printf("#NET,wifi,down,%s\r\n", why);
  // Whatever was waiting in RAM cannot be sent now and must not sit there
  // being coalesced over, so it goes to flash immediately, oldest first.
  pendingToRing("link_down");
  return false;
}

void NetLogger::appendToRing(const dg::StoredRecord& r) {
  if (!slots_.ready()) {
    // No filesystem this boot: live uploads keep working through the pending
    // queue, but anything that needed buffering is genuinely lost and is
    // counted into field 8 rather than quietly discarded.
    ++stats_.droppedNoRing;
    return;
  }
  const uint32_t before = ring_.stats().overwritten;
  ring_.append(r);
  const uint32_t after = ring_.stats().overwritten;
  if (after != before) {
    stats_.droppedRingOverwrite += (after - before);
  }
  stats_.ringCount = ring_.count();
  // One line per flash write, so an outage is visible in the serial log rather
  // than only showing up as a ring= count later.
  Serial.printf("#NET,ring,append,count=%u\r\n",
                static_cast<unsigned>(ring_.count()));
}

dg::LinkState NetLogger::linkState() const {
  // Only a usable link counts as ONLINE: associated, and past the point where
  // uploads actually go out.
  return (state_ == NetState::ONLINE || state_ == NetState::FLUSHING)
             ? dg::LinkState::ONLINE
             : dg::LinkState::OFFLINE;
}

void NetLogger::pendingToRing(const char* why) {
  if (pending_.empty()) {
    return;
  }
  const uint8_t moved = pending_.count();
  dg::StoredRecord sr{};
  while (pending_.popOldest(sr)) { // oldest first, so order is preserved
    appendToRing(sr);
  }
  Serial.printf("#NET,pending,to_ring,n=%u,why=%s,ring=%u\r\n",
                static_cast<unsigned>(moved), why,
                static_cast<unsigned>(ring_.count()));
}

void NetLogger::drainQueue() {
  if (queue_ == nullptr) {
    return;
  }

  const dg::RouteParams routeParams = {dg::PendingQueue::kCapacity,
                                       kPendingHoldMs, cfg::UPLOAD_SPACING_MS};

  dg::StoredRecord sr{};
  while (xQueueReceive(static_cast<QueueHandle_t>(queue_), &sr, 0) == pdTRUE) {
    const dg::LinkState link = linkState();
    const uint32_t pendingAge =
        pending_.empty() ? 0u : (millis() - oldestPendingMs_);

    const dg::Route route =
        dg::routeRecord(routeParams, link, ring_.count(), pending_.count(),
                        timeSynced_, pendingAge);

    if (route == dg::Route::RING) {
      appendToRing(sr);
      continue;
    }

    const bool allowCoalesce =
        dg::mayCoalesce(routeParams, link, sr.rec.event == dg::kEventSnapshot,
                        pending_.newestIsSnapshot(),
                        millis() - lastUploadMs_);

    bool coalesced = false;
    if (!pending_.push(sr, allowCoalesce, coalesced)) {
      appendToRing(sr); // RAM buffer full: spill rather than drop
      continue;
    }
    if (coalesced) {
      // The snapshot it replaced was never uploaded, so it is accounted for
      // like any other record that left without being sent (D65).
      ++stats_.droppedCoalesced;
    } else if (pending_.count() == 1) {
      oldestPendingMs_ = millis();
    }
  }
}

void NetLogger::agePending() {
  if (pending_.empty()) {
    return;
  }
  if ((millis() - oldestPendingMs_) < kPendingHoldMs) {
    return;
  }
  // Held its allowance without being sent - usually still waiting for the
  // clock. Move the lot to flash, in order, where it can be dated later (D52).
  pendingToRing("aged");
}

bool NetLogger::postBody(const char* path, const char* contentType,
                         const char* body, uint16_t len, char* resp,
                         uint16_t respCap, int& httpCode) {
  WiFiClient client;
  HTTPClient http;
  httpCode = 0;
  if (resp != nullptr && respCap > 0) {
    resp[0] = '\0';
  }

  if (!http.begin(client, kHost, kPort, path)) {
    return false;
  }
  http.setTimeout(kHttpTimeoutMs);
  http.setConnectTimeout(kHttpTimeoutMs);
  http.addHeader("Content-Type", contentType);

  httpCode = http.POST(reinterpret_cast<uint8_t*>(const_cast<char*>(body)),
                       len);
  bool ok = false;
  if (httpCode == 200) {
    const String payload = http.getString();
    if (resp != nullptr && respCap > 0) {
      const uint16_t n = payload.length() < (respCap - 1)
                             ? static_cast<uint16_t>(payload.length())
                             : static_cast<uint16_t>(respCap - 1);
      for (uint16_t i = 0; i < n; ++i) {
        resp[i] = payload[i];
      }
      resp[n] = '\0';
    }
    ok = true;
  }
  http.end();
  return ok;
}

bool NetLogger::uploadOneLive() {
  dg::StoredRecord sr{};
  if (!pending_.peekOldest(sr)) {
    return false;
  }

  const dg::EpochFate fate = dg::resolveEpoch(sr, bootId_, millis(),
                                              nowEpoch(), timeSynced_);
  if (fate == dg::EpochFate::WAIT) {
    return false; // agePending() will ring it if this lasts
  }
  if (fate == dg::EpochFate::DROP_OLD_BOOT) {
    pending_.popOldest(sr);
    ++stats_.droppedOldBoot;
    return false;
  }

  const uint16_t len = dg::buildUpdateBody(g_formBuf, sizeof(g_formBuf),
                                           writeKey_, sr, droppedTotal());
  if (len == 0) {
    pending_.popOldest(sr);
    ++stats_.uploadsFailed;
    return false;
  }

  int code = 0;
  lastUploadMs_ = millis();
  const bool http200 =
      postBody("/update", "application/x-www-form-urlencoded", g_formBuf, len,
               g_respBuf, sizeof(g_respBuf), code);

  // ThingSpeak answers 200 with the new entry id, or "0" when it rejected the
  // update - so a 200 alone is not success.
  const bool accepted = http200 && g_respBuf[0] != '\0' &&
                        !(g_respBuf[0] == '0' && g_respBuf[1] == '\0');

  if (accepted) {
    pending_.popOldest(sr);
    ++stats_.uploadsOk;
    Serial.printf("#NET,up,ok,id=%s\r\n", g_respBuf);
    if (!pending_.empty()) {
      oldestPendingMs_ = millis();
    }
    return true;
  }

  ++stats_.uploadsFailed;
  Serial.printf("#NET,up,fail,code=%d\r\n", code);
  // Failed: park it on flash so ordering and durability are preserved (D51).
  pending_.popOldest(sr);
  appendToRing(sr);
  return false;
}

bool NetLogger::flushRing() {
  // Shed leading records that can never be dated (D8).
  while (ring_.count() > 0) {
    dg::StoredRecord head{};
    if (!ring_.peekOldest(head)) {
      break;
    }
    if (dg::resolveEpoch(head, bootId_, millis(), nowEpoch(), timeSynced_) !=
        dg::EpochFate::DROP_OLD_BOOT) {
      break;
    }
    ring_.dropOldest(1);
    ++stats_.droppedOldBoot;
    Serial.printf("#NET,drop,oldboot,ring=%u\r\n",
                  static_cast<unsigned>(ring_.count()));
  }
  stats_.ringCount = ring_.count();
  if (ring_.count() == 0) {
    persistCounters();
    return false;
  }

  dg::BulkBuilder builder(g_bulkBuf, kBulkBufBytes, writeKey_);
  uint16_t n = 0;
  const uint16_t held = ring_.count();
  for (uint16_t i = 0; i < held && n < kMaxBulkEntries; ++i) {
    dg::StoredRecord sr{};
    if (!ring_.readAt(i, sr)) {
      break;
    }
    const dg::EpochFate fate = dg::resolveEpoch(sr, bootId_, millis(),
                                                nowEpoch(), timeSynced_);
    if (fate == dg::EpochFate::WAIT ||
        fate == dg::EpochFate::DROP_OLD_BOOT) {
      break; // stop the batch here; the next window deals with it
    }
    if (!builder.addEntry(sr, droppedTotal())) {
      break;
    }
    ++n;
  }
  if (n == 0 || !builder.finish()) {
    return false;
  }

  snprintf(g_pathBuf, sizeof(g_pathBuf), "/channels/%s/bulk_update.json",
           channelId_);

  int code = 0;
  lastUploadMs_ = millis();
  const bool http200 =
      postBody(g_pathBuf, "application/json", builder.body(),
               builder.length(), g_respBuf, sizeof(g_respBuf), code);
  const bool accepted = http200 && (strstr(g_respBuf, "success") != nullptr);

  if (!accepted) {
    ++stats_.uploadsFailed;
    Serial.printf("#NET,bulk,n=%u,fail,code=%d\r\n", static_cast<unsigned>(n),
                  code);
    return false;
  }

  ring_.dropOldest(n);
  ++stats_.bulkFlushes;
  stats_.uploadsOk += n;
  stats_.ringCount = ring_.count();
  Serial.printf("#NET,bulk,n=%u,ok,ring=%u\r\n", static_cast<unsigned>(n),
                static_cast<unsigned>(ring_.count()));
  persistCounters();
  return true;
}

void NetLogger::persistCounters() {
  // Both are no-ops unless the value moved, so this is a handful of NVS writes
  // a day (D48/D53).
  store_.saveAck(ring_.lastDroppedSeq());
  store_.saveDropped(droppedTotal());
}

void NetLogger::taskLoop() {
  stateSinceMs_ = millis();
  lastUploadMs_ = millis() - cfg::UPLOAD_SPACING_MS; // first upload can go at once

  for (;;) {
    stats_.stackHighWater =
        uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
    stats_.freeHeap = ESP.getFreeHeap();
    stats_.minFreeHeap = ESP.getMinFreeHeap();
    stats_.state = state_;
    stats_.timeSynced = timeSynced_;

    switch (state_) {
      case NetState::FS_MOUNT: {
        const bool fsOk = slots_.begin(cfg::RING_CAPACITY);
        stats_.fsReady = fsOk;
        stats_.fsFormatted = slots_.formatted();
        if (fsOk) {
          ring_.mount(lastAckSeq_);
          stats_.ringCount = ring_.count();
          Serial.printf(
              "#NET,fs,mount,ok=1,formatted=%u,slots=%u,count=%u,bad=%u,"
              "acked=%lu\r\n",
              static_cast<unsigned>(slots_.formatted() ? 1 : 0),
              static_cast<unsigned>(cfg::RING_CAPACITY),
              static_cast<unsigned>(ring_.count()),
              static_cast<unsigned>(ring_.stats().badSlots),
              static_cast<unsigned long>(ring_.stats().ackSkipped));
        } else {
          Serial.print("#NET,fs,mount,ok=0\r\n");
        }
        // bootId and the dropped baseline come from NVS; never log the key.
        Serial.printf("#NET,boot,id=%u,dropped=%lu,ack=%lu,channel=%s\r\n",
                      static_cast<unsigned>(bootId_),
                      static_cast<unsigned long>(droppedBase_),
                      static_cast<unsigned long>(lastAckSeq_), channelId_);
        state_ = NetState::DISCONNECTED;
        stateSinceMs_ = millis();
        break;
      }

      case NetState::DISCONNECTED:
        if (netEnabled_ && (millis() - lastAttemptMs_) >= backoffMs_) {
          lastAttemptMs_ = millis();
          WiFi.mode(WIFI_STA);
          WiFi.begin(ssid_, pass_);
          state_ = NetState::CONNECTING;
          stateSinceMs_ = millis();
          Serial.printf("#NET,wifi,connecting,ssid=%s\r\n", ssid_);
        }
        break;

      case NetState::CONNECTING:
        if (!netEnabled_) {
          goOffline("debug_off");
        } else if (WiFi.status() == WL_CONNECTED) {
          backoffMs_ = kBackoffMinMs;
          stats_.wifiUp = true;
          Serial.printf("#NET,wifi,up,ip=%s\r\n",
                        WiFi.localIP().toString().c_str());
          configTime(cfg::GMT_OFFSET_S, cfg::DST_OFFSET_S, cfg::NTP_SERVER);
          lastNtpMs_ = millis();
          state_ = NetState::SYNCING_TIME;
          stateSinceMs_ = millis();
        } else if ((millis() - stateSinceMs_) >= kConnectTimeoutMs) {
          backoffMs_ = backoffMs_ * 2 > kBackoffMaxMs ? kBackoffMaxMs
                                                      : backoffMs_ * 2;
          goOffline("connect_timeout");
        }
        break;

      case NetState::SYNCING_TIME:
        if (!netEnabled_) {
          goOffline("debug_off");
        } else if (WiFi.status() != WL_CONNECTED) {
          goOffline("link_lost");
        } else if (nowEpoch() != 0) {
          timeSynced_ = true;
          state_ = NetState::ONLINE;
          stateSinceMs_ = millis();
          Serial.printf("#NET,ntp,ok,epoch=%lu\r\n",
                        static_cast<unsigned long>(nowEpoch()));
        } else if ((millis() - stateSinceMs_) >= kSyncTimeoutMs) {
          // Online but undated: records wait, then go to the ring (D52).
          state_ = NetState::ONLINE;
          stateSinceMs_ = millis();
          Serial.print("#NET,ntp,timeout\r\n");
        }
        break;

      case NetState::ONLINE:
      case NetState::FLUSHING:
        if (!netEnabled_) {
          goOffline("debug_off");
          break;
        }
        if (WiFi.status() != WL_CONNECTED) {
          goOffline("link_lost");
          break;
        }
        if ((millis() - lastNtpMs_) >= kNtpResyncMs) {
          configTime(cfg::GMT_OFFSET_S, cfg::DST_OFFSET_S, cfg::NTP_SERVER);
          lastNtpMs_ = millis();
        }
        if (!timeSynced_ && nowEpoch() != 0) {
          timeSynced_ = true;
          Serial.printf("#NET,ntp,late,epoch=%lu\r\n",
                        static_cast<unsigned long>(nowEpoch()));
        }
        // A backlog always wins, so the channel stays in order (D51).
        state_ = ring_.count() > 0 ? NetState::FLUSHING : NetState::ONLINE;
        if (spacingElapsed() && timeSynced_) {
          if (state_ == NetState::FLUSHING) {
            flushRing();
          } else {
            uploadOneLive();
          }
        }
        break;
    }

    drainQueue();
    agePending();
    stats_.ringCount = ring_.count();

    vTaskDelay(pdMS_TO_TICKS(kTaskTickMs));
  }
}

} // namespace hal
