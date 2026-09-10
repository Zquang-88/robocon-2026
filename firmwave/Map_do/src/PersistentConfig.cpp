#include "PersistentConfig.h"

#include <string.h>

namespace {
constexpr char NVS_NAMESPACE[] = "mechanism";
constexpr char NVS_KEY[] = "config";

void setReason(char *reason, size_t capacity, const char *text) {
  if (!reason || capacity == 0) return;
  strncpy(reason, text, capacity - 1);
  reason[capacity - 1] = '\0';
}
}  // namespace

bool PersistentConfig::begin() {
  if (!ready_) ready_ = preferences_.begin(NVS_NAMESPACE, false);
  return ready_;
}

bool PersistentConfig::load(MechanismConfig &config,
                            char *reason, size_t reasonCapacity) {
  if (!begin()) {
    setReason(reason, reasonCapacity, "NVS_OPEN");
    return false;
  }
  if (preferences_.getBytesLength(NVS_KEY) != sizeof(MechanismConfig)) {
    setReason(reason, reasonCapacity, "NVS_EMPTY_OR_SIZE");
    return false;
  }
  MechanismConfig candidate{};
  if (preferences_.getBytes(NVS_KEY, &candidate, sizeof(candidate)) !=
      sizeof(candidate)) {
    setReason(reason, reasonCapacity, "NVS_READ");
    return false;
  }
  if (candidate.crc32 != mechanismConfigCrc(candidate)) {
    setReason(reason, reasonCapacity, "NVS_CRC");
    return false;
  }
  if (!validateMechanismConfig(candidate, reason, reasonCapacity)) return false;
  config = candidate;
  setReason(reason, reasonCapacity, "NVS_OK");
  return true;
}

bool PersistentConfig::save(MechanismConfig &config,
                            char *reason, size_t reasonCapacity) {
  if (!begin()) {
    setReason(reason, reasonCapacity, "NVS_OPEN");
    return false;
  }
  if (!validateMechanismConfig(config, reason, reasonCapacity)) return false;
  config.crc32 = mechanismConfigCrc(config);
  if (preferences_.putBytes(NVS_KEY, &config, sizeof(config)) != sizeof(config)) {
    setReason(reason, reasonCapacity, "NVS_WRITE");
    return false;
  }
  MechanismConfig verified{};
  if (preferences_.getBytes(NVS_KEY, &verified, sizeof(verified)) !=
      sizeof(verified)) {
    setReason(reason, reasonCapacity, "NVS_VERIFY_READ");
    return false;
  }
  if (memcmp(&verified, &config, sizeof(config)) != 0 ||
      verified.crc32 != mechanismConfigCrc(verified) ||
      !validateMechanismConfig(verified, reason, reasonCapacity)) {
    setReason(reason, reasonCapacity, "NVS_VERIFY_MISMATCH");
    return false;
  }
  config = verified;
  setReason(reason, reasonCapacity, "NVS_SAVED_VERIFIED");
  return true;
}

bool PersistentConfig::erase(char *reason, size_t reasonCapacity) {
  if (!begin()) {
    setReason(reason, reasonCapacity, "NVS_OPEN");
    return false;
  }
  const bool ok = preferences_.remove(NVS_KEY);
  setReason(reason, reasonCapacity, ok ? "NVS_ERASED" : "NVS_NOT_PRESENT");
  return ok;
}
