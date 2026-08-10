
#ifndef SERIALCHANNEL_HPP
#define SERIALCHANNEL_HPP

#include "Messages.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cstring>
#include <string>

class SerialChannel {
private:
  std::string buff_;
  MotorCommand lastMotorCmd_;

  // validate and parse a single line, log if error
  void parseLine_(std::string line) {
    MotorCommand cmd;
    if (cmd.deserialze(line) == -1) {

      Serial.write(
          LogMessage("Invalid motor command: " + line).serialize().c_str());
      return;
    }
    lastMotorCmd_ = cmd;
  }

public:
  // read into buffer, parse last line, and leave any partial line in the
  // buffer
  void read() {
    // read into buffer
    int bytesToRead = std::min(Serial.available(), 63);
    if (bytesToRead <= 0) {
      return;
    }
    char buff[64] = {};
    Serial.readBytes(buff, bytesToRead);

    buff_.append(buff);

    // retrieve the last newline in the buffer, parse it and erase everything up
    // to it, leaving any partial line in the buffer
    size_t lastNl = buff_.rfind('\n');
    if (lastNl != std::string::npos) {
      size_t prevNl =
          (lastNl == 0) ? std::string::npos : buff_.rfind('\n', lastNl - 1);
      size_t lineStart = (prevNl != std::string::npos) ? prevNl + 1 : 0;
      parseLine_(buff_.substr(lineStart, lastNl - lineStart));
      buff_.erase(0, lastNl + 1); // +1 to remove the nl char itself
    }

    // instructions are small, so if the buffer grows too large it must be
    // garbage
    if (buff_.size() > 63) {
      Serial.write(
          LogMessage("Buffer overflow, clearing buffer").serialize().c_str());
      buff_.clear();
    }
  }

  // writes a message to the serial port
  // @param message: the message to send, must implement ASendInterface
  void write(const ASendMessage &message) {
    Serial.write(message.serialize().c_str());
  }

  // TODO: make sure the caller of the function checks the timestamp and stops
  // motors if stale
  // get the last motor command
  MotorCommand getMotorCmd() { return lastMotorCmd_; }
};

#endif
