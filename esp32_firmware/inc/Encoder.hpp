#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <cstdint>

class Encoder {
private:
  int8_t _dir;
  unsigned long _pulse_count;
  unsigned long _last_pulse_count;

  uint8_t _signal1_pin;
  uint8_t _signal2_pin;

public:
  Encoder(uint8_t signal1_pin, uint8_t signal2_pin)
      : _dir(1), _pulse_count(0), _last_pulse_count(0),
        _signal1_pin(signal1_pin), _signal2_pin(signal2_pin) {}

  Encoder(const Encoder &) = delete;
  Encoder &operator=(const Encoder &) = delete;

  // setting up pins and interrupts - should be called in main setup()
  void setup() {
    pinMode(_signal1_pin, INPUT_PULLUP);
    pinMode(_signal2_pin, INPUT_PULLUP);
    attachInterrupt(
        _signal1_pin, [this]() -> void { on_signal1_raise(); }, RISING);
  }

  // interrupt callback incrementing pulse count and updating direction
  void on_signal1_raise() {
    if (digitalRead(_signal2_pin) == LOW) {
      _dir = 1;
    } else {
      _dir = -1;
    }
    _pulse_count++;
  }

  // returns the amount of pulses since the last function call
  unsigned long get_pulse_diff() {
    unsigned long snaps = _pulse_count;
    // this substraction even works when _pulse_count overflows (since the
    // result is casted to unsigned again)
    unsigned long diff = snaps - _last_pulse_count;
    _last_pulse_count = snaps;

    return diff;
  }
};

#endif
