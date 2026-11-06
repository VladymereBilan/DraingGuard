// The logger's three persisted values, in the same NVS namespace as the
// controller's baseline.
//
//   bootId       incremented once per boot; stamped into every record so an
//                undatable record from an earlier boot can be recognised (D8)
//   lastAckSeq   highest ring seq already delivered, so a reboot never
//                re-sends what ThingSpeak already has (D53)
//   droppedTotal cumulative field-8 counter, surviving reboots because it is
//                the data-integrity ledger for the trial runs (D48)
//
// Writes are rare by design: bootId once per boot, the other two only when
// their value actually changes.

#ifndef DG_BOOT_ID_H
#define DG_BOOT_ID_H

#include <stdint.h>

namespace hal {

struct NetPersist {
  uint16_t bootId;
  uint32_t lastAckSeq;
  uint32_t droppedTotal;
};

class NetStore {
public:
  NetStore();

  // Increments and stores bootId, then returns all three values.
  NetPersist beginBoot();

  // Tells a second NetStore what is already on flash, so its first save() is
  // not a pointless rewrite. Used by the logger, which does not call
  // beginBoot() - that must happen exactly once per boot.
  void seed(uint32_t lastAckSeq, uint32_t droppedTotal);

  // No-ops when the value is unchanged, which is what keeps flash wear down.
  void saveAck(uint32_t lastAckSeq);
  void saveDropped(uint32_t droppedTotal);

  uint32_t writes() const { return writes_; }

private:
  uint32_t lastAckWritten_;
  uint32_t lastDroppedWritten_;
  uint32_t writes_;
};

} // namespace hal

#endif // DG_BOOT_ID_H
