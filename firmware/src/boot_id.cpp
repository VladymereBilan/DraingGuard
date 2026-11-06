#include "boot_id.h"

#include <Preferences.h>

namespace hal {
namespace {

constexpr char kNamespace[] = "dg";
constexpr char kKeyBootId[] = "boot_id";
constexpr char kKeyAckSeq[] = "ack_seq";
constexpr char kKeyDropped[] = "dropped";

} // namespace

NetStore::NetStore()
    : lastAckWritten_(0), lastDroppedWritten_(0), writes_(0) {}

NetPersist NetStore::beginBoot() {
  NetPersist p{};

  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    // No NVS: run with volatile counters rather than refusing to boot.
    p.bootId = 1;
    return p;
  }

  p.bootId = static_cast<uint16_t>(prefs.getUShort(kKeyBootId, 0) + 1);
  prefs.putUShort(kKeyBootId, p.bootId);
  p.lastAckSeq = prefs.getUInt(kKeyAckSeq, 0);
  p.droppedTotal = prefs.getUInt(kKeyDropped, 0);
  prefs.end();

  ++writes_;
  lastAckWritten_ = p.lastAckSeq;
  lastDroppedWritten_ = p.droppedTotal;
  return p;
}

void NetStore::seed(uint32_t lastAckSeq, uint32_t droppedTotal) {
  lastAckWritten_ = lastAckSeq;
  lastDroppedWritten_ = droppedTotal;
}

void NetStore::saveAck(uint32_t lastAckSeq) {
  if (lastAckSeq == lastAckWritten_) {
    return;
  }
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    return;
  }
  prefs.putUInt(kKeyAckSeq, lastAckSeq);
  prefs.end();
  lastAckWritten_ = lastAckSeq;
  ++writes_;
}

void NetStore::saveDropped(uint32_t droppedTotal) {
  if (droppedTotal == lastDroppedWritten_) {
    return;
  }
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    return;
  }
  prefs.putUInt(kKeyDropped, droppedTotal);
  prefs.end();
  lastDroppedWritten_ = droppedTotal;
  ++writes_;
}

} // namespace hal
