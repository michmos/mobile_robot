#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <cstdint>

class Encoder {
private:
  volatile int32_t tick_count_;
  volatile int8_t sign_;

  uint8_t signal1_pin_;
  uint8_t signal2_pin_;

  // interrupt callback incrementing tick count when FORWARD and decrementing
  // when BACKWARD
  void IRAM_ATTR on_signal1_raise_() {
    tick_count_ += (digitalRead(signal2_pin_) == LOW) ? sign_ : -sign_;
  }

  static void IRAM_ATTR isr_(void *arg) {
    static_cast<Encoder *>(arg)->on_signal1_raise_();
  }

public:
  Encoder(uint8_t signal1_pin, uint8_t signal2_pin)
      : tick_count_(0), signal1_pin_(signal1_pin), signal2_pin_(signal2_pin),
        sign_(1) {}

  Encoder(const Encoder &) = delete;
  Encoder &operator=(const Encoder &) = delete;

  // setting up pins and interrupts - should be called in main setup()
  void setup() {
    pinMode(signal1_pin_, INPUT_PULLUP);
    pinMode(signal2_pin_, INPUT_PULLUP);
    attachInterruptArg(signal1_pin_, isr_, this, RISING);
  }

  // flips counting direction of encoder
  void setInverted(bool inverted) { sign_ = (inverted) ? -1 : 1; }

  int32_t get_tick_count() const { return tick_count_; }
};

#endif
