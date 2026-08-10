
#ifndef SERIALCHANNEL_HPP
#define SERIALCHANNEL_HPP

#include "Messages.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cstring>
#include <string>

class SerialChannel {
private:
  // bytes pulled off the uart per read() call
  static const int k_read_chunk = 128;

  // longest partial line kept before assuming the buffer is garbage
  static const size_t k_max_line_len = 256;

  std::string buff_;
  msgs::MotorCmd lastMotorCmd_;
  msgs::Config config_;

  // validate and parse a single line, log if error
  // overrides lastMotorCmd_ and config_ if respective line found
  void parseLine_(std::string line) {
    // trim end
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
      line.pop_back();
    }

    if (line.empty()) {
      return;
    }

    switch (line[0]) {
    case 'M': {
      msgs::MotorCmd cmd;
      if (cmd.init(line) == -1) {
        writeln(msgs::Log("Invalid motor command: " + line));
        return;
      }
      lastMotorCmd_ = cmd;
    } break;
    case 'C': {
      msgs::Config config;
      if (config.init(line) == -1) {
        writeln(msgs::Log("Invalid configuration message: " + line));
        return;
      }
      config_ = config;
    } break;
    default: {
      writeln(msgs::Log("Invalid line: " + line));
    } break;
    }
  }

public:
  // read into buffer, parse lines, and leave any partial line in the
  // buffer
  // @return: number of bytes read
  size_t read() {
    // read into buffer
    int available = Serial.available();
    if (available <= 0) {
      return 0;
    }
    int bytesToRead = (available > k_read_chunk) ? k_read_chunk : available;

    char buff[k_read_chunk] = {};
    size_t bytesRead = Serial.readBytes(buff, bytesToRead);

    buff_.append(buff, bytesRead);

    // parse all complete lines, erase them and leave remainder for next time
    size_t nlChar;
    while ((nlChar = buff_.find('\n')) != std::string::npos) {
      parseLine_(buff_.substr(0, nlChar));
      buff_.erase(0, nlChar + 1); // +1 to remove the nl char itself
    }

    // only a partial line is left at this point, so anything longer than the
    // longest legal message must be garbage
    if (buff_.size() > k_max_line_len) {
      writeln(msgs::Log("Buffer overflow, clearing buffer"));
      buff_.clear();
    }
    return bytesRead;
  }

  // writes a message to the serial port
  // @param message: the message to send, must implement ASendInterface
  void write(const msgs::ASendMessage &message) {
    Serial.write(message.serialize().c_str());
  }

  // writes a message to the serial port. Add a nl character if not already
  // present
  // @param message: the message to send, must implement ASendInterface
  void writeln(const msgs::ASendMessage &message) {
    std::string serialized = message.serialize();
    if (serialized.back() != '\n') {
      serialized += "\n";
    }
    Serial.write(serialized.c_str());
  }

  // get the last motor command
  msgs::MotorCmd getMotorCmd() const { return lastMotorCmd_; }
  msgs::Config getConfig() const { return config_; }
};

#endif
