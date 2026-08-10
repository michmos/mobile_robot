#ifndef MESSAGES_HPP
#define MESSAGES_HPP

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

  unsigned long parseULong_(const char *str, bool *errFlag) {
    if (!str) {
      *errFlag = true;
      return 0;
    }

    char *endptr = NULL;
    unsigned long val = strtoul(str, &endptr, 10);
    if (endptr == str || *endptr != '\0') {
      *errFlag = true;
      return 0;
    }
    return val;
  }

  bool parseBool_(const char *str, bool *errFlag) {
    unsigned long val = parseULong_(str, errFlag);
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
// esp32 reboots - the esp32 keeps no persistent copy
class Config : public AReceiveMessage {
public:
  bool initialized;

  // set when the message was parsed
  unsigned long timestamp_ms;

  // PI gains - shared by both wheels
  float kp;
  float ki;
  float integral_limit; // anti-windup clamp on |integral term|, duty fraction

  // feedforward - per wheel, since stiction and torque differ normally
  float slope_left;     // duty fraction per (rad/s)
  float slope_right;    // duty fraction per (rad/s)
  float min_duty_left;  // deadband compensation, duty fraction [0,1)
  float min_duty_right; // deadband compensation, duty fraction [0,1)

  // limits and scaling
  float ticks_per_wheel_rev; // counted encoder edges per wheel revolution,

  // timing
  uint16_t watchdog_timeout_ms; // stop motors if no MotorCmd within this
  uint16_t control_rate_hz;     // PI update rate
  uint16_t report_rate_hz;      // EncoderData publish rate

  // sign conventions - the two motors are mounted mirrored, so "forwards" is
  // opposite rotation on each side
  bool invert_left;
  bool invert_right;

  // defaults
  Config()
      : initialized(false), timestamp_ms(0), kp(0.0f), ki(0.0f),
        integral_limit(0.0f), slope_left(0.1f), slope_right(0.1f),
        min_duty_left(0.0f), min_duty_right(0.0f), ticks_per_wheel_rev(480.0f),
        watchdog_timeout_ms(200), control_rate_hz(200), report_rate_hz(50),
        invert_left(false), invert_right(false) {}

  int init(const std::string &line) override {
    std::string copy = line;

    const char *id = strtok(&copy[0], ",");
    if (!id || strlen(id) != 1 || id[0] != 'C') {
      return -1;
    }

    Config parsed;
    bool errFlag = false;

    parsed.kp = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.ki = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.integral_limit = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.slope_left = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.slope_right = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.min_duty_left = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.min_duty_right = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.ticks_per_wheel_rev = parseFloat_(strtok(NULL, ","), &errFlag);
    parsed.watchdog_timeout_ms = parseULong_(strtok(NULL, ","), &errFlag);
    parsed.control_rate_hz = parseULong_(strtok(NULL, ","), &errFlag);
    parsed.report_rate_hz = parseULong_(strtok(NULL, ","), &errFlag);
    parsed.invert_left = parseBool_(strtok(NULL, ","), &errFlag);
    parsed.invert_right = parseBool_(strtok(NULL, ","), &errFlag);

    if (errFlag) {
      return -1;
    }

    // there shouldn't be any leftover tokens
    if (strtok(NULL, ",") != NULL) {
      return -1;
    }

    if (!parsed.isValid_()) {
      return -1;
    }

    parsed.initialized = true;
    parsed.timestamp_ms = millis();
    *this = parsed;
    return 0;
  }

private:
  // range check
  bool isValid_() const {
    return kp >= 0.0f && ki >= 0.0f && integral_limit >= 0.0f &&
           slope_left > 0.0f && slope_right > 0.0f && min_duty_left >= 0.0f &&
           min_duty_left < 1.0f && min_duty_right >= 0.0f &&
           min_duty_right < 1.0f && ticks_per_wheel_rev > 0.0f &&
           watchdog_timeout_ms > 0 && control_rate_hz > 0 &&
           report_rate_hz > 0 && report_rate_hz <= control_rate_hz;
  }
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

// S,<event_string>
class Event : public ASendMessage {
public:
  enum e_event_type {
    SETUP, // starting setup
    ACK,   // acknowledge receipt of configuration data
  } e;

  Event(e_event_type e) : e(e) {}
  std::string serialize() const override {
    std::string enumString;
    switch (e) {
    case SETUP:
      enumString = "SETUP";
      break;
    case ACK:
      enumString = "ACK";
      break;
    }

    return std::string("S,") + enumString + "\n";
  }
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
