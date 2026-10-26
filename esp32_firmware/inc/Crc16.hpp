#ifndef CRC16_HPP
#define CRC16_HPP

#include <cstddef>
#include <cstdint>

// checksum of the Config ("C,...") message, see Messages.hpp for the wire
// format. Shared by both ends of the serial link like ConfigPayload.hpp

namespace protocol {

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
// Check value: crc16("123456789") == 0x29B1
inline uint16_t crc16(const char *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(data[i]) << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

} // namespace protocol

#endif
