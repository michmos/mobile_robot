#ifndef MOBILE_ROBOT_HARDWARE_WIRE_PROTOCOL_HPP
#define MOBILE_ROBOT_HARDWARE_WIRE_PROTOCOL_HPP

// pi side of the serial protocol with the esp32, see
// esp32_firmware/inc/Messages.hpp for the wire format

// shared with the firmware, the include path points at esp32_firmware/
#include "inc/ConfigPayload.hpp"

#include <cstdint>
#include <string>

namespace mobile_robot_hardware {

// the esp32 -> pi lines are told apart by these, they have to match what
// Event, Log and EncoderData in esp32_firmware/inc/Messages.hpp write
// (test/test_wire_contract.cpp checks that)
inline constexpr char k_setup_line[] = "S,SETUP"; // the whole line
inline constexpr char k_ack_prefix[] = "S,ACK,";
inline constexpr char k_nack_prefix[] = "S,NACK,";
inline constexpr char k_log_prefix[] = "L,";
inline constexpr char k_encoder_prefix[] = "E,";

// C,<handshake_id>,<kp>,...,<invert_right>,<crc>\n
// floats are written with enough digits that the esp32 parses back exactly
// the value the pi validated, <crc> lets it detect a corrupted line
std::string serializeConfig(const protocol::ConfigPayload &payload,
                            uint32_t handshakeId);

// M,<left_vel_cmd>,<right_vel_cmd>\n, velocities in rad/s
std::string serializeMotorCmd(float leftVelCmd, float rightVelCmd);

// E,<left_ticks>,<right_ticks>,<us>\n - see EncoderData in
// esp32_firmware/inc/Messages.hpp. `line` has its trailing '\n' stripped
// @throws std::runtime_error if the line is malformed
void parseEncoderReport(const std::string &line, int32_t &leftTicks,
                        int32_t &rightTicks, uint32_t &timestampUs);

// splits a handshake reply "<prefix><handshake id>[,<rest>]"
// the id has to be plain decimal digits and fit into uint32_t
// @return: false if the line has another prefix or no valid id
bool parseHandshakeReply(const std::string &line, const std::string &prefix,
                         uint32_t &id, std::string &rest);

} // namespace mobile_robot_hardware

#endif
