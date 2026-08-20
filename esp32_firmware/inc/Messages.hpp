#ifndef MESSAGES_HPP
#define MESSAGES_HPP

#include "ConfigPayload.hpp"
#include <Arduino.h>
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

// C,<kp>,<ki>,<integral_limit>,<slope_left>,<slope_right>,<min_duty_left>,
//   <min_duty_right>,<ticks_per_wheel_rev>,
//   <watchdog_timeout_ms>,<control_rate_hz>,<report_rate_hz>,
//   <invert_left>,<invert_right>\n
//
// sent once by the pi after it sees the SETUP event, and re-sent whenever the
// esp32 reboots
//
// the payload fields and their range check live in ConfigPayload.hpp, which the
// pi shares, so both ends agree on what a valid configuration is
class Config : public AReceiveMessage, public protocol::ConfigPayload {
public:
  bool initialized;

  // set when the message was parsed
  unsigned long timestamp_ms;

  Config() : initialized(false), timestamp_ms(0), lastError_(NULL) {}

  int init(const std::string &line) override {
    std::string copy = line;

    const char *id = strtok(&copy[0], ",");
    if (!id || strlen(id) != 1 || id[0] != 'C') {
      lastError_ = k_malformed;
      return -1;
    }

    bool errFlag = false;
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
    timestamp_ms = millis();
    lastError_ = NULL;
    return 0;
  }

  // why the last init() failed, NULL if it succeeded. Points at a string
  // literal, so it stays valid for as long as the object does
  const char *lastError() const { return lastError_; }

private:
  static constexpr const char *k_malformed = "malformed";

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
class Event : public ASendMessage {
public:
  enum e_event_type {
    SETUP, // starting setup
    ACK,   // acknowledge receipt of configuration data
    NACK,  // configuration rejected, detail names the offending field
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
