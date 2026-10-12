#ifndef MESSAGES_HPP
#define MESSAGES_HPP

#include "ConfigPayload.hpp"
#include "Crc16.hpp"
#include <Arduino.h>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

// this file describes the communication protocol between the pi and the esp32,
// in both directions

///////////////////////////////////////////////////////////////////////////////
// pi -> esp32
///////////////////////////////////////////////////////////////////////////////

namespace msgs {
class AReceiveMessage {
protected:
  float parseFloat_(const char *str, bool *errFlag) {
    if (!str) {
      *errFlag = true;
      return -1.0f;
    }

    char *endptr = NULL;
    float val = strtof(str, &endptr);
    if (endptr == str || *endptr != '\0') {
      *errFlag = true;
      return -1.0f;
    }
    return val;
  }

  // parses wide and range checks before narrowing, a value above UINT16_MAX
  // would otherwise wrap silently
  uint16_t parseUInt16_(const char *str, bool *errFlag) {
    if (!str) {
      *errFlag = true;
      return 0;
    }

    char *endptr = NULL;
    unsigned long val = strtoul(str, &endptr, 10);
    if (endptr == str || *endptr != '\0' || val > UINT16_MAX) {
      *errFlag = true;
      return 0;
    }
    return (uint16_t)val;
  }

  uint32_t parseUInt32_(const char *str, bool *errFlag) {
    if (!str) {
      *errFlag = true;
      return 0;
    }

    char *endptr = NULL;
    unsigned long long val = strtoull(str, &endptr, 10);
    if (endptr == str || *endptr != '\0' || val == 0 || val > UINT32_MAX) {
      *errFlag = true;
      return 0;
    }
    return (uint32_t)val;
  }

  bool parseBool_(const char *str, bool *errFlag) {
    uint16_t val = parseUInt16_(str, errFlag);
    if (val > 1) {
      *errFlag = true;
      return false;
    }
    return val == 1;
  }

public:
  virtual int init(const std::string &line) = 0;
};

// M,<left_vel_cmd>,<right_vel_cmd>\n
class MotorCmd : public AReceiveMessage {
public:
  float left_vel_cmd;  // rad/s
  float right_vel_cmd; // rad/s
  unsigned long timestamp_ms;

  MotorCmd() : left_vel_cmd(0), right_vel_cmd(0), timestamp_ms(0) {}
  MotorCmd(float left, float right)
      : left_vel_cmd(left), right_vel_cmd(right), timestamp_ms(millis()) {}

  int init(const std::string &line) {
    std::string copy = line;

    timestamp_ms = millis();

    const char *id = strtok(&copy[0], ",");
    if (!id || strlen(id) != 1 || id[0] != 'M') {
      return -1;
    }

    bool errFlag = false;
    left_vel_cmd = parseFloat_(strtok(NULL, ","), &errFlag);
    right_vel_cmd = parseFloat_(strtok(NULL, ","), &errFlag);
    if (errFlag) {
      return -1;
    }
    return 0;
  }
};

// C,<handshake_id>,<kp>,<ki>,<integral_limit>,<slope_left>,<slope_right>,
//   <min_duty_left>,<min_duty_right>,<ticks_per_wheel_rev>,
//   <watchdog_timeout_ms>,<control_rate_hz>,<report_rate_hz>,
//   <invert_left>,<invert_right>,<crc>\n
//
// <crc> is the CRC-16 (Crc16.hpp) of everything before its comma, as 4
// uppercase hex digits. A config that passes it is exactly what the pi sent,
// so an ACK means the esp32 runs the pi's values
//
// the pi starts a handshake when it configures, when a SETUP arrives while no
// handshake is running, and when parameters change. It re-sends the config
// until the esp32 ACKs the same handshake_id (a NACK is retried as well).
// All re-sends of one handshake carry the same non-zero handshake_id and the
// same payload, a new handshake gets a new, higher handshake_id. So ACKing a
// re-send of an already applied handshake_id is correct
//
// the payload fields and their range check live in ConfigPayload.hpp, which the
// pi shares, so both ends agree on what a valid configuration is
class Config : public AReceiveMessage, public protocol::ConfigPayload {
public:
  bool initialized;

  // chosen by the pi, 0 until a message was parsed
  uint32_t handshake_id;

  Config() : initialized(false), handshake_id(0), lastError_(NULL) {}

  int init(const std::string &line) override {
    // the checksum covers everything before the last comma
    const size_t crcSep = line.rfind(',');
    const bool crcOk = checksumOk_(line, crcSep);
    std::string copy =
        crcSep == std::string::npos ? line : line.substr(0, crcSep);

    const char *id = strtok(&copy[0], ",");
    if (!id || strlen(id) != 1 || id[0] != 'C') {
      lastError_ = k_malformed;
      return -1;
    }

    bool errFlag = false;
    // parsed first so a NACK can name the handshake even if the rest is bad
    handshake_id = parseUInt32_(strtok(NULL, ","), &errFlag);
    if (!crcOk) {
      lastError_ = k_checksum;
      return -1;
    }
    protocol::ConfigPayload parsed;
    parsed.kp = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.ki = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.integral_limit = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.slope_left = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.slope_right = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.min_duty_left = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.min_duty_right = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.ticks_per_wheel_rev = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.watchdog_timeout_ms = parseUInt16_(strtok(NULL, ","), &errFlag);
    parsed.control_rate_hz = parseUInt16_(strtok(NULL, ","), &errFlag);
    parsed.report_rate_hz = parseUInt16_(strtok(NULL, ","), &errFlag);
    parsed.invert_left = parseBool_(strtok(NULL, ","), &errFlag);
    parsed.invert_right = parseBool_(strtok(NULL, ","), &errFlag);

    // check for error during parsing and remaining tokens
    if (errFlag || strtok(NULL, ",") != NULL) {
      lastError_ = k_malformed;
      return -1;
    }

    // range check parsed values
    protocol::e_config_field ret = protocol::validateConfig(parsed);
    if (ret != protocol::CONFIG_OK) {
      lastError_ = protocol::configFieldName(ret);
      return -1;
    }

    static_cast<protocol::ConfigPayload &>(*this) = parsed;
    initialized = true;
    lastError_ = NULL;
    return 0;
  }

  // why the last init() failed, NULL if it succeeded. Points at a string
  // literal, so it stays valid for as long as the object does
  const char *lastError() const { return lastError_; }

private:
  static constexpr const char *k_malformed = "malformed";
  static constexpr const char *k_checksum = "checksum";

  // @param sep: position of the comma before the checksum
  // @return: true if the 4 hex digits after sep match the CRC of line[0, sep)
  static bool checksumOk_(const std::string &line, size_t sep) {
    if (sep == std::string::npos || line.size() - sep - 1 != 4) {
      return false;
    }
    uint16_t received = 0;
    for (size_t i = sep + 1; i < line.size(); ++i) {
      const char c = line[i];
      if (!isxdigit(static_cast<unsigned char>(c))) {
        return false;
      }
      received = static_cast<uint16_t>(
          (received << 4) |
          (isdigit(static_cast<unsigned char>(c)) ? c - '0'
                                                   : (toupper(c) - 'A' + 10)));
    }
    return received == protocol::crc16(line.c_str(), sep);
  }

  const char *lastError_;
};

///////////////////////////////////////////////////////////////////////////////
// esp32 -> pi
///////////////////////////////////////////////////////////////////////////////

class ASendMessage {
public:
  virtual std::string serialize() const = 0;
};

// E,<left_ticks_count>,<right_ticks_count>,<us>\n
class EncoderData : public ASendMessage {
public:
  int32_t left_ticks_count;
  int32_t right_ticks_count;
  unsigned long us;

  EncoderData() : left_ticks_count(0), right_ticks_count(0), us(0) {}
  EncoderData(int32_t left, int32_t right, uint32_t dt)
      : left_ticks_count(left), right_ticks_count(right), us(dt) {}

  std::string serialize() const override {
    return "E," + std::to_string(left_ticks_count) + "," +
           std::to_string(right_ticks_count) + "," + std::to_string(us) + "\n";
  }
};

// I,<qx>,<qy>,<qz>,<qw>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>\n
class ImuData : public ASendMessage {
public:
  float qx;
  float qy;
  float qz;
  float qw;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;

  ImuData()
      : qx(0), qy(0), qz(0), qw(1), gx(0), gy(0), gz(0), ax(0), ay(0), az(0) {}
  ImuData(float qx, float qy, float qz, float qw, float gx, float gy, float gz,
          float ax, float ay, float az)
      : qx(qx), qy(qy), qz(qz), qw(qw), gx(gx), gy(gy), gz(gz), ax(ax), ay(ay),
        az(az) {}

  std::string serialize() const override {
    return "I," + std::to_string(qx) + "," + std::to_string(qy) + "," +
           std::to_string(qz) + "," + std::to_string(qw) + "," +
           std::to_string(gx) + "," + std::to_string(gy) + "," +
           std::to_string(gz) + "," + std::to_string(ax) + "," +
           std::to_string(ay) + "," + std::to_string(az) + "\n";
  }
};

// S,<event_string>[,<detail>]
// ACK detail: <handshake_id>, NACK detail: <handshake_id>,<field>
class Event : public ASendMessage {
public:
  enum e_event_type {
    SETUP, // unconfigured, sent as a beacon until a configuration is applied
    ACK,   // configuration with the handshake_id in detail passed the CRC and
           // range check. Sent on parse, updateConfig() applies it in the
           // same loop() pass (or skips it if it already runs)
    NACK,  // configuration with the handshake_id in detail was rejected, the
           // detail after it names the offending field, or "checksum" /
           // "malformed"
  } e;

  Event(e_event_type e) : e(e), detail_() {}
  Event(e_event_type e, const std::string &detail) : e(e), detail_(detail) {}

  std::string serialize() const override {
    std::string enumString;
    switch (e) {
    case SETUP:
      enumString = "SETUP";
      break;
    case ACK:
      enumString = "ACK";
      break;
    case NACK:
      enumString = "NACK";
      break;
    }

    std::string detail = detail_.empty() ? "" : ("," + detail_);
    return std::string("S,") + enumString + detail + "\n";
  }

private:
  std::string detail_;
};

// L,<log message>
class Log : public ASendMessage {
public:
  std::string log_message;

  Log(const std::string &msg) : log_message(msg) {}
  std::string serialize() const override { return "L," + log_message + "\n"; }
};
} // namespace msgs

#endif
