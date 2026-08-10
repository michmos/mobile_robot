#ifndef MESSAGES_HPP
#define MESSAGES_HPP

#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include <string>

// this file describes the communication protocol between the pi and the esp32,
// in both directions

///////////////////////////////////////////////////////////////////////////////
// pi -> esp32
///////////////////////////////////////////////////////////////////////////////

// C,<left_vel_cmd>,<right_vel_cmd>\n
class MotorCommand {
public:
  float left_vel_cmd;
  float right_vel_cmd;
  unsigned long timestamp_ms;

  MotorCommand() : left_vel_cmd(0), right_vel_cmd(0), timestamp_ms(0) {}
  MotorCommand(float left, float right)
      : left_vel_cmd(left), right_vel_cmd(right), timestamp_ms(millis()) {}

  int deserialze(const std::string &line) {
    std::string copy = line;

    timestamp_ms = millis();

    const char *id = strtok(&copy[0], ",");
    if (!id || strlen(id) != 1 || id[0] != 'C') {
      return -1;
    }

    bool error = false;
    auto parseFloat = [&error](const char *str) -> float {
      if (!str) {
        error = true;
        return -1.0f;
      }

      char *endptr = NULL;
      float val = strtof(str, &endptr);
      if (endptr == str || *endptr != '\0') {
        error = true;
        return -1.0f;
      }
      return val;
    };

    left_vel_cmd = parseFloat(strtok(NULL, ","));
    right_vel_cmd = parseFloat(strtok(NULL, ","));
    if (error) {
      return -1;
    }
    return 0;
  }
};

///////////////////////////////////////////////////////////////////////////////
// esp32 -> pi
///////////////////////////////////////////////////////////////////////////////

class ASendMessage {
public:
  virtual std::string serialize() const = 0;
};

// E,<left_ticks_delta>,<right_ticks_delta>,<dt_us>\n
class EncoderData : public ASendMessage {
public:
  int32_t left_ticks_delta;
  int32_t right_ticks_delta;
  uint32_t dt_us;

  EncoderData() : left_ticks_delta(0), right_ticks_delta(0), dt_us(0) {}
  EncoderData(int32_t left, int32_t right, uint32_t dt)
      : left_ticks_delta(left), right_ticks_delta(right), dt_us(dt) {}

  std::string serialize() const override {
    return "E," + std::to_string(left_ticks_delta) + "," +
           std::to_string(right_ticks_delta) + "," + std::to_string(dt_us) +
           "\n";
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

// L,<log message>
class LogMessage : public ASendMessage {
public:
  std::string log_message;

  LogMessage(const std::string &msg) : log_message(msg) {}
  std::string serialize() const override { return "L," + log_message + "\n"; }
};

#endif
