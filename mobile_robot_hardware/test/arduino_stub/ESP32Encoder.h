#ifndef ESP32ENCODER_STUB_H
#define ESP32ENCODER_STUB_H

// host stand-in for the ESP32Encoder library (madhephaestus/ESP32Encoder, the
// PCNT quadrature counter esp32_firmware/inc/Encoder.hpp is built on). Only
// the subset the firmware uses. Instead of counting edges, each attached
// encoder records how it was set up and returns a count the test sets, both
// through arduino_stub (setEncoderCount(), encoderUnit()). The state is
// cleared by arduino_stub::reset() like the rest of the stub

#include "Arduino.h"

#include <cstdint>
#include <map>

enum class encType { single, half, full };

enum class puType { up, down, none };

namespace arduino_stub {

// what an ESP32Encoder was attached with, plus its count
struct EncoderUnit {
  int a_pin = -1;
  int b_pin = -1;
  encType type = encType::single;
  puType pull = puType::none; // useInternalWeakPullResistors at attach time
  uint16_t filter = 0;        // last setFilter() value, 0 = off
  int64_t count = 0;          // what getCount() returns
};

struct EncoderState {
  std::map<int, EncoderUnit> units; // per A pin, only attached encoders
  // ESP32Encoder::useInternalWeakPullResistors, pull-down by default like the
  // library
  puType pull = puType::down;
};

inline EncoderState encoderState;

inline void resetEncoders() { encoderState = EncoderState(); }
inline const bool k_encoderResetRegistered =
    (resetHooks().push_back(&resetEncoders), true);

// @return: whether an encoder with A on a_pin is attached
inline bool encoderAttached(int a_pin) {
  return encoderState.units.count(a_pin) != 0;
}
// @return: the unit attached with A on a_pin, default constructed if none
inline EncoderUnit encoderUnit(int a_pin) {
  auto it = encoderState.units.find(a_pin);
  return it == encoderState.units.end() ? EncoderUnit() : it->second;
}
// sets the library count (signed 64 bit) of the encoder with A on a_pin, as
// if the hardware had counted to it
// @return: false if no encoder is attached there
inline bool setEncoderCount(int a_pin, int64_t count) {
  auto it = encoderState.units.find(a_pin);
  if (it == encoderState.units.end()) {
    return false;
  }
  it->second.count = count;
  return true;
}

} // namespace arduino_stub

class ESP32Encoder {
private:
  int a_pin_ = -1;

  arduino_stub::EncoderUnit *unit_() {
    auto it = arduino_stub::encoderState.units.find(a_pin_);
    return it == arduino_stub::encoderState.units.end() ? nullptr : &it->second;
  }

  void attach_(int a, int b, encType type) {
    if (isAttached()) {
      return; // the library logs "already attached" and keeps the old unit
    }
    arduino_stub::EncoderUnit u;
    u.a_pin = a;
    u.b_pin = b;
    u.type = type;
    u.pull = useInternalWeakPullResistors;
    u.filter = 250; // the library's attach() sets this filter
    arduino_stub::encoderState.units[a] = u;
    a_pin_ = a;
  }

public:
  // a reference into the stub state, so arduino_stub::reset() restores the
  // default. Assigned like the library's static member
  static inline puType &useInternalWeakPullResistors =
      arduino_stub::encoderState.pull;

  ESP32Encoder() = default;

  void attachFullQuad(int a, int b) { attach_(a, b, encType::full); }
  void attachHalfQuad(int a, int b) { attach_(a, b, encType::half); }
  void attachSingleEdge(int a, int b) { attach_(a, b, encType::single); }

  bool isAttached() { return unit_() != nullptr; }

  // 0 while not attached, like the library
  int64_t getCount() {
    auto *u = unit_();
    return u ? u->count : 0;
  }

  void setCount(int64_t value) {
    if (auto *u = unit_()) {
      u->count = value;
    }
  }

  int64_t clearCount() {
    setCount(0);
    return 0;
  }

  // the library ignores it while not attached and clamps to 1023
  void setFilter(uint16_t value) {
    if (auto *u = unit_()) {
      u->filter = value > 1023 ? 1023 : value;
    }
  }
};

#endif
