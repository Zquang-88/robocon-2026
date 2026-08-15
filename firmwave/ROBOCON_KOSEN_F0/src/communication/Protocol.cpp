#include "Protocol.h"

#include <string.h>

namespace rbt2 {

namespace {
constexpr uint8_t MAGIC_R = 0x52;
constexpr uint8_t MAGIC_B = 0x42;
constexpr size_t CRC_OFFSET = offsetof(PacketHeader, crc16);

uint16_t packetCrc(const uint8_t *packet, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    const uint8_t value = (i == CRC_OFFSET || i == CRC_OFFSET + 1)
                              ? 0
                              : packet[i];
    crc ^= static_cast<uint16_t>(value) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}
} // namespace

uint16_t crc16Ccitt(const uint8_t *data, size_t length, uint16_t initial) {
  uint16_t crc = initial;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

size_t encodePacket(PacketType type, uint32_t sequence, uint32_t timestampMs,
                    const void *payload, uint16_t payloadLength,
                    uint8_t *output, size_t outputCapacity) {
  if (!output || payloadLength > MAX_PAYLOAD ||
      outputCapacity < sizeof(PacketHeader) + payloadLength ||
      (payloadLength && !payload))
    return 0;

  PacketHeader header{};
  header.magic = MAGIC;
  header.version = VERSION;
  header.type = static_cast<uint8_t>(type);
  header.payloadLength = payloadLength;
  header.sequence = sequence;
  header.timestampMs = timestampMs;
  header.crc16 = 0;

  memcpy(output, &header, sizeof(header));
  if (payloadLength)
    memcpy(output + sizeof(header), payload, payloadLength);

  const size_t total = sizeof(header) + payloadLength;
  header.crc16 = packetCrc(output, total);
  memcpy(output + CRC_OFFSET, &header.crc16, sizeof(header.crc16));
  return total;
}

StreamParser::StreamParser(PacketHandler handler, void *context)
    : handler_(handler), context_(context) {}

void StreamParser::reset() {
  state_ = State::MagicR;
  received_ = 0;
  expected_ = sizeof(PacketHeader);
}

void StreamParser::beginHeader() {
  buffer_[0] = MAGIC_R;
  buffer_[1] = MAGIC_B;
  received_ = 2;
  expected_ = sizeof(PacketHeader);
  state_ = State::Header;
}

void StreamParser::feed(uint8_t byte) {
  switch (state_) {
    case State::MagicR:
      if (byte == MAGIC_R) state_ = State::MagicB;
      else ++stats_.discardedBytes;
      break;

    case State::MagicB:
      if (byte == MAGIC_B) beginHeader();
      else if (byte != MAGIC_R) {
        state_ = State::MagicR;
        ++stats_.discardedBytes;
      }
      break;

    case State::Header:
      buffer_[received_++] = byte;
      if (received_ == sizeof(PacketHeader)) validateHeader();
      break;

    case State::Payload:
      buffer_[received_++] = byte;
      if (received_ == expected_) finishPacket();
      break;
  }
}

void StreamParser::feed(const uint8_t *data, size_t length) {
  if (!data) return;
  for (size_t i = 0; i < length; ++i) feed(data[i]);
}

void StreamParser::validateHeader() {
  PacketHeader header{};
  memcpy(&header, buffer_, sizeof(header));
  if (header.magic != MAGIC) {
    reset();
    return;
  }
  if (header.version != VERSION) {
    ++stats_.versionErrors;
    reset();
    return;
  }
  if (header.payloadLength > MAX_PAYLOAD) {
    ++stats_.lengthErrors;
    reset();
    return;
  }

  expected_ = sizeof(PacketHeader) + header.payloadLength;
  if (header.payloadLength == 0) finishPacket();
  else state_ = State::Payload;
}

void StreamParser::finishPacket() {
  PacketHeader header{};
  memcpy(&header, buffer_, sizeof(header));
  if (packetCrc(buffer_, expected_) != header.crc16) {
    ++stats_.crcErrors;
    reset();
    return;
  }

  ++stats_.validPackets;
  if (handler_) {
    const PacketView view{header, buffer_ + sizeof(PacketHeader)};
    handler_(view, context_);
  }
  reset();
}

} // namespace rbt2
