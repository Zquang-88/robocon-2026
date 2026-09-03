#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../../src/communication/Protocol.h"

struct Capture {
  int count = 0;
  rbt2::PacketHeader header{};
  uint8_t payload[64] = {};
};

void capturePacket(const rbt2::PacketView &packet, void *context) {
  Capture &capture = *static_cast<Capture *>(context);
  ++capture.count;
  capture.header = packet.header;
  if (packet.header.payloadLength)
    memcpy(capture.payload, packet.payload, packet.header.payloadLength);
}

int main() {
  static_assert(sizeof(rbt2::PacketHeader) == 16, "header layout changed");

  rbt2::HelloPayload hello{2, 2, 0, 0x12345678};
  uint8_t encoded[128] = {};
  const size_t length = rbt2::encodePacket(
      rbt2::PacketType::Hello, 42, 1000, &hello, sizeof(hello), encoded,
      sizeof(encoded));
  assert(length == sizeof(rbt2::PacketHeader) + sizeof(hello));

  Capture capture;
  rbt2::StreamParser parser(capturePacket, &capture);

  // Noise and a fragmented packet must resynchronize.
  const uint8_t noise[] = {0x00, 0xFF, 0x52, 0x00};
  parser.feed(noise, sizeof(noise));
  parser.feed(encoded, 3);
  parser.feed(encoded + 3, length - 3);
  assert(capture.count == 1);
  assert(capture.header.sequence == 42);
  assert(memcmp(capture.payload, &hello, sizeof(hello)) == 0);

  // Back-to-back packets must both be emitted.
  parser.feed(encoded, length);
  parser.feed(encoded, length);
  assert(capture.count == 3);

  // Corruption must be rejected without losing the next valid packet.
  encoded[length - 1] ^= 0x01;
  parser.feed(encoded, length);
  assert(capture.count == 3);
  assert(parser.stats().crcErrors == 1);
  encoded[length - 1] ^= 0x01;
  parser.feed(encoded, length);
  assert(capture.count == 4);

  const char *standard = "123456789";
  assert(rbt2::crc16Ccitt(reinterpret_cast<const uint8_t *>(standard), 9) ==
         0x29B1);

  printf("RBT/2 protocol tests passed: packets=%lu crcErrors=%lu\n",
         static_cast<unsigned long>(parser.stats().validPackets),
         static_cast<unsigned long>(parser.stats().crcErrors));
  return 0;
}
