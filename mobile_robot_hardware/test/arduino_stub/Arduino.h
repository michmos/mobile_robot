#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H

// host stand-in for the Arduino core (esp32 core 2.0.x API), so the hardware
// independent firmware code in esp32_firmware/inc/ compiles and runs in gtest.
// Everything the firmware touches is faked here and can be driven and
// inspected from the test through the arduino_stub namespace. This is the
// single shared stub for all firmware tests, extend it instead of adding a
// second one. Call arduino_stub::reset() at the start of every test (from the
// fixture's SetUp()) so no state leaks between tests

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <string>

///////////////////////////////////////////////////////////////////////////////
// constants and esp32 specific keywords
///////////////////////////////////////////////////////////////////////////////

// the arduino core defines these as macros. Constants instead, so they can't
// clash with names in gtest or the standard library. Values match the esp32
// core
constexpr uint8_t LOW = 0x0;
constexpr uint8_t HIGH = 0x1;

constexpr uint8_t INPUT = 0x01;
constexpr uint8_t OUTPUT = 0x03;
constexpr uint8_t PULLUP = 0x04;
constexpr uint8_t INPUT_PULLUP = 0x05;
constexpr uint8_t PULLDOWN = 0x08;
constexpr uint8_t INPUT_PULLDOWN = 0x09;

constexpr int RISING = 0x01;
constexpr int FALLING = 0x02;
constexpr int CHANGE = 0x03;

// places a function in IRAM on the esp32, meaningless on the host
#define IRAM_ATTR

// the host tests are single threaded, so critical sections are no-ops
struct portMUX_TYPE {
  uint32_t owner;
  uint32_t count;
};
#define portMUX_INITIALIZER_UNLOCKED                                           \
  { 0, 0 }
inline void portENTER_CRITICAL(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL(portMUX_TYPE *) {}

///////////////////////////////////////////////////////////////////////////////
// test side: fakes and their state
///////////////////////////////////////////////////////////////////////////////

namespace arduino_stub {

// fake of the uart behind the arduino Serial object. The test queues the bytes
// the firmware will receive with pushInput() and reads what the firmware sent
// with output()
class FakeSerial {
private:
  std::deque<char> input_;
  std::string output_;
  unsigned long baud_ = 0;

public:
  // ---- firmware side, the subset of HardwareSerial the firmware uses ----
  void begin(unsigned long baud) { baud_ = baud; }

  int available() { return static_cast<int>(input_.size()); }

  // like the real one, returns early with fewer bytes if the queue runs dry
  // (the real one waits for its timeout first, the fake doesn't block)
  size_t readBytes(char *buffer, size_t length) {
    size_t n = 0;
    while (n < length && !input_.empty()) {
      buffer[n++] = input_.front();
      input_.pop_front();
    }
    return n;
  }
  size_t readBytes(uint8_t *buffer, size_t length) {
    return readBytes(reinterpret_cast<char *>(buffer), length);
  }

  int read() {
    if (input_.empty()) {
      return -1;
    }
    const int c = static_cast<unsigned char>(input_.front());
    input_.pop_front();
    return c;
  }

  size_t write(uint8_t c) {
    output_ += static_cast<char>(c);
    return 1;
  }
  size_t write(const uint8_t *buffer, size_t size) {
    output_.append(reinterpret_cast<const char *>(buffer), size);
    return size;
  }
  size_t write(const char *str) {
    if (!str) {
      return 0;
    }
    const size_t len = strlen(str);
    output_.append(str, len);
    return len;
  }

  size_t print(const char *str) { return write(str); }
  size_t print(const std::string &str) { return write(str.c_str()); }
  size_t print(char c) { return write(static_cast<uint8_t>(c)); }
  size_t print(int val) { return print(std::to_string(val)); }
  size_t print(unsigned int val) { return print(std::to_string(val)); }
  size_t print(long val) { return print(std::to_string(val)); }
  size_t print(unsigned long val) { return print(std::to_string(val)); }

  // the arduino core terminates lines with \r\n
  size_t println() { return write("\r\n"); }
  template <typename T> size_t println(const T &val) {
    const size_t n = print(val);
    return n + println();
  }

  explicit operator bool() const { return true; }

  // ---- test side ----
  // queue bytes for the firmware to read
  void pushInput(const std::string &bytes) {
    input_.insert(input_.end(), bytes.begin(), bytes.end());
  }

  // everything the firmware wrote since the last reset()/clearOutput()
  const std::string &output() const { return output_; }
  void clearOutput() { output_.clear(); }

  // baud rate passed to begin(), 0 if begin() wasn't called
  unsigned long baud() const { return baud_; }

  void reset() {
    input_.clear();
    output_.clear();
    baud_ = 0;
  }
};

// state of one ledc (pwm) channel, last values passed to the ledc functions
struct LedcChannel {
  uint32_t freq_hz = 0;
  uint8_t resolution_bits = 0;
  uint32_t duty = 0;
  int pin = -1; // last pin attached to this channel, -1 if none
  bool setup = false;
  bool written = false;
};

// an interrupt registered with attachInterrupt()/attachInterruptArg()
struct Interrupt {
  void (*isr)() = nullptr;
  void (*isrArg)(void *) = nullptr;
  void *arg = nullptr;
  int mode = 0;
};

// all state of the fake core, one instance for the whole test binary
struct State {
  // a single 64 bit clock like the esp32's esp_timer, micros() and millis()
  // are derived from it like on the real core
  uint64_t micros = 0;

  std::map<uint8_t, uint8_t> pinModes;      // last pinMode() per pin
  std::map<uint8_t, uint8_t> digitalWrites; // last digitalWrite() per pin
  std::map<uint8_t, int> digitalReads;      // what digitalRead() returns
  std::map<uint8_t, LedcChannel> ledc;      // per channel
  std::map<uint8_t, Interrupt> interrupts;  // per pin
};

inline State state;

// ---- clock ----
// micros() and millis() truncate the clock to 32 bit like on the esp32, so
// they wrap at 2^32 (micros() after ~71.6 min, millis() after ~49.7 days)
// independently of each other. unsigned long is 64 bit on the host, so code
// that has to survive the wrap must do its arithmetic in uint32_t (as it
// would on the esp32, where unsigned long is 32 bit)
inline void setMicros(uint64_t us) { state.micros = us; }
inline void setMillis(uint64_t ms) { state.micros = ms * 1000ULL; }
inline void advanceMicros(uint64_t us) { state.micros += us; }
inline void advanceMillis(uint64_t ms) { state.micros += ms * 1000ULL; }

// ---- gpio ----
// sets what digitalRead(pin) returns, LOW for pins never set
inline void setDigitalRead(uint8_t pin, int value) {
  state.digitalReads[pin] = value;
}
// @return: last mode passed to pinMode(pin), -1 if never called
inline int pinModeOf(uint8_t pin) {
  auto it = state.pinModes.find(pin);
  return it == state.pinModes.end() ? -1 : it->second;
}
// @return: last value passed to digitalWrite(pin), -1 if never called
inline int digitalWritten(uint8_t pin) {
  auto it = state.digitalWrites.find(pin);
  return it == state.digitalWrites.end() ? -1 : it->second;
}

// ---- ledc ----
// @return: state of a pwm channel, default constructed if never touched
inline LedcChannel ledcChannel(uint8_t channel) {
  auto it = state.ledc.find(channel);
  return it == state.ledc.end() ? LedcChannel() : it->second;
}

// ---- interrupts ----
inline bool hasInterrupt(uint8_t pin) {
  return state.interrupts.count(pin) != 0;
}
// @return: mode the interrupt on pin was attached with, 0 if none
inline int interruptMode(uint8_t pin) {
  auto it = state.interrupts.find(pin);
  return it == state.interrupts.end() ? 0 : it->second.mode;
}
// runs the isr attached to pin as if its edge had occurred
// @return: false if no isr is attached to pin
inline bool fireInterrupt(uint8_t pin) {
  auto it = state.interrupts.find(pin);
  if (it == state.interrupts.end()) {
    return false;
  }
  if (it->second.isrArg) {
    it->second.isrArg(it->second.arg);
  } else if (it->second.isr) {
    it->second.isr();
  }
  return true;
}

} // namespace arduino_stub

// the arduino Serial object
inline arduino_stub::FakeSerial Serial;

namespace arduino_stub {
// clears all state of the fake core: clock back to 0, serial queues, pin,
// pwm and interrupt records
inline void reset() {
  state = State();
  Serial.reset();
}
} // namespace arduino_stub

///////////////////////////////////////////////////////////////////////////////
// firmware side: the arduino core api
///////////////////////////////////////////////////////////////////////////////

inline unsigned long micros() {
  return static_cast<uint32_t>(arduino_stub::state.micros);
}
inline unsigned long millis() {
  return static_cast<uint32_t>(arduino_stub::state.micros / 1000ULL);
}

// advances the fake clock instead of blocking, so code that waits in a loop
// sees time pass without the test sleeping
inline void delay(uint32_t ms) { arduino_stub::advanceMillis(ms); }
inline void delayMicroseconds(uint32_t us) { arduino_stub::advanceMicros(us); }

inline void pinMode(uint8_t pin, uint8_t mode) {
  arduino_stub::state.pinModes[pin] = mode;
}
inline void digitalWrite(uint8_t pin, uint8_t val) {
  arduino_stub::state.digitalWrites[pin] = val;
}
inline int digitalRead(uint8_t pin) {
  auto it = arduino_stub::state.digitalReads.find(pin);
  return it == arduino_stub::state.digitalReads.end() ? LOW : it->second;
}

// esp32 core 2.0.x ledc api (removed in 3.x)
inline uint32_t ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution) {
  arduino_stub::LedcChannel &c = arduino_stub::state.ledc[channel];
  c.freq_hz = freq;
  c.resolution_bits = resolution;
  c.setup = true;
  return freq;
}
inline void ledcAttachPin(uint8_t pin, uint8_t channel) {
  arduino_stub::state.ledc[channel].pin = pin;
}
inline void ledcWrite(uint8_t channel, uint32_t duty) {
  arduino_stub::LedcChannel &c = arduino_stub::state.ledc[channel];
  c.duty = duty;
  c.written = true;
}

inline void attachInterrupt(uint8_t pin, void (*isr)(), int mode) {
  arduino_stub::Interrupt i;
  i.isr = isr;
  i.mode = mode;
  arduino_stub::state.interrupts[pin] = i;
}
inline void attachInterruptArg(uint8_t pin, void (*isr)(void *), void *arg,
                               int mode) {
  arduino_stub::Interrupt i;
  i.isrArg = isr;
  i.arg = arg;
  i.mode = mode;
  arduino_stub::state.interrupts[pin] = i;
}
inline void detachInterrupt(uint8_t pin) {
  arduino_stub::state.interrupts.erase(pin);
}

#endif
