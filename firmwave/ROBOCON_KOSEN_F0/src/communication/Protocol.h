#pragma once

#include <stddef.h>
#include <stdint.h>

namespace rbt2 {

constexpr uint16_t MAGIC = 0x4252; // Bytes on wire: 'R', 'B'.
constexpr uint8_t VERSION = 2;
constexpr uint16_t MAX_PAYLOAD = 1024;

enum class PacketType : uint8_t {
  Hello = 1,
  HelloAck = 2,
  Heartbeat = 3,
  Telemetry = 4,
  ConfigRead = 5,
  ConfigData = 6,
  ConfigWrite = 7,
  ConfigSave = 8,
  Command = 9,
  Ack = 10,
  Error = 11,
  StreamSample = 12
};

#pragma pack(push, 1)
struct PacketHeader {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t payloadLength;
  uint32_t sequence;
  uint32_t timestampMs;
  uint16_t crc16;
};

struct HelloPayload {
  uint8_t minimumVersion;
  uint8_t maximumVersion;
  uint16_t reserved;
  uint32_t clientNonce;
};

struct HelloAckPayload {
  uint32_t clientNonce;
  uint32_t firmwareVersion;
  uint32_t configVersion;
  uint32_t capabilities;
};

struct HeartbeatPayload {
  uint8_t mode;
  uint8_t state;
  uint16_t reserved;
  uint32_t faultFlags;
  uint32_t lastRxSequence;
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 16, "RBT/2 header must be 16 bytes");

struct PacketView {
  PacketHeader header;
  const uint8_t *payload;
};

struct ParserStats {
  uint32_t validPackets = 0;
  uint32_t crcErrors = 0;
  uint32_t lengthErrors = 0;
  uint32_t versionErrors = 0;
  uint32_t discardedBytes = 0;
};

using PacketHandler = void (*)(const PacketView &packet, void *context);

uint16_t crc16Ccitt(const uint8_t *data, size_t length,
                    uint16_t initial = 0xFFFF);

// Returns encoded byte count, or 0 when the output buffer is too small.
size_t encodePacket(PacketType type, uint32_t sequence, uint32_t timestampMs,
                    const void *payload, uint16_t payloadLength,
                    uint8_t *output, size_t outputCapacity);

class StreamParser {
 public:
  StreamParser(PacketHandler handler, void *context = nullptr);

  void feed(uint8_t byte);
  void feed(const uint8_t *data, size_t length);
  void reset();
  const ParserStats &stats() const { return stats_; }

 private:
  enum class State : uint8_t { MagicR, MagicB, Header, Payload };

  State state_ = State::MagicR;
  uint8_t buffer_[sizeof(PacketHeader) + MAX_PAYLOAD] = {};
  size_t received_ = 0;
  size_t expected_ = sizeof(PacketHeader);
  PacketHandler handler_ = nullptr;
  void *context_ = nullptr;
  ParserStats stats_;

  void beginHeader();
  void validateHeader();
  void finishPacket();
};

} // namespace rbt2
