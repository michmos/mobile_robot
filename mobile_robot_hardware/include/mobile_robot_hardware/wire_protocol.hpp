#ifndef MOBILE_ROBOT_HARDWARE_WIRE_PROTOCOL_HPP
#define MOBILE_ROBOT_HARDWARE_WIRE_PROTOCOL_HPP

// pi side of the serial protocol with the esp32, see
// esp32_firmware/inc/Messages.hpp for the wire format

// shared with the firmware, the include path points at esp32_firmware/
#include "inc/ConfigPayload.hpp"

#include <cstdint>
#include <string>

namespace mobile_robot_hardware {

// C,<handshake_id>,<kp>,...,<invert_right>\n
// floats are written with enough digits that the esp32 parses back exactly
// the value the pi validated
std::string serializeConfig(const protocol::ConfigPayload &payload,
                            uint32_t handshakeId);

// splits a handshake reply "<prefix><handshake id>[,<rest>]"
// the id has to be plain decimal digits and fit into uint32_t
// @return: false if the line has another prefix or no valid id
bool parseHandshakeReply(const std::string &line, const std::string &prefix,
                         uint32_t &id, std::string &rest);

} // namespace mobile_robot_hardware

#endif
