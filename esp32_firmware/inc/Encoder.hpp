#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <cstdint>

// quadrature encoder on the esp32's PCNT counter (ESP32Encoder library),
// 4x decoding: a glitch on one channel counts +1 and -1 and cancels out
class Encoder {
public:
  // PCNT glitch filter in APB cycles (12.5 ns): 400 = 5 us, well below the
  // ~50 us between real edges at full motor speed
  static constexpr uint16_t k_glitch_filter_apb_cycles = 400;

private:
  // getCount() is not const in the library
  mutable ESP32Encoder pcnt_;
  int8_t sign_;
  // keeps the reported count continuous when the sign changes
  int64_t offset_;

  uint8_t signal1_pin_;
  uint8_t signal2_pin_;

  // the library counts down while A leads B, reported here as forward (+)
  int64_t forwardCount_() const { return -pcnt_.getCount(); }

  int64_t count_() const { return sign_ * forwardCount_() + offset_; }

public:
  // stores the pins only: global objects are constructed before the core is up
  Encoder(uint8_t signal1_pin, uint8_t signal2_pin)
      : sign_(1), offset_(0), signal1_pin_(signal1_pin),
        signal2_pin_(signal2_pin) {}

  // the PCNT unit belongs to this object
  Encoder(const Encoder &) = delete;
  Encoder &operator=(const Encoder &) = delete;

  // attaches a PCNT unit in full quadrature - should be called in main setup()
  void setup() {
    // open collector outputs; the library defaults to pull-down. Static,
    // read by attach*()
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    pcnt_.attachFullQuad(signal1_pin_, signal2_pin_);
    pcnt_.setFilter(k_glitch_filter_apb_cycles);
  }

  // flips the counting direction of later ticks, the count keeps its value
  void setInverted(bool inverted) {
    const int8_t sign = inverted ? -1 : 1;
    if (sign == sign_) {
      return;
    }
    // sign_ * raw + offset_ == sign * raw + new offset
    offset_ += 2 * sign_ * forwardCount_();
    sign_ = sign;
  }

  // narrowed to the int32 wire format. It wraps, the pi only uses differences
  int32_t get_tick_count() const {
    return static_cast<int32_t>(static_cast<uint32_t>(count_()));
  }
};

#endif
