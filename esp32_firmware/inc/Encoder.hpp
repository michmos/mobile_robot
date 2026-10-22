#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <cstdint>

// quadrature encoder on the esp32's PCNT hardware counter (ESP32Encoder
// library). Full quadrature (4x) decoding counts every edge of A and B, +1 or
// -1 by the level of the other channel, so a glitch on one channel (an extra
// rise + fall) cancels out instead of adding up in one direction (#69)
class Encoder {
public:
  // PCNT glitch filter in APB cycles (12.5 ns each, the library clamps to
  // 1023 = 12.8 us): 400 = 5 us, pulses shorter than that are ignored.
  // Edge spacing at full speed, 4x decoding with ~1964 counts/wheel rev:
  // - xacro command limit 10 rad/s = 1.6 rev/s -> ~3100 edges/s, 320 us apart
  // - bound for the motor itself 30 rad/s (~290 rpm, above the ~200 rpm the
  //   motors reach) -> ~9400 edges/s, 107 us apart, ~53 us if the A/B phase
  //   error halves the shorter gap
  // 5 us stays well below a quarter of that (~13 us), so no real edge is lost
  static constexpr uint16_t k_glitch_filter_apb_cycles = 400;

private:
  // getCount() is not const in the library
  mutable ESP32Encoder pcnt_;
  int8_t sign_;
  // keeps the reported count continuous when the sign changes
  int64_t offset_;

  uint8_t signal1_pin_;
  uint8_t signal2_pin_;

  // the library counts down while A leads B (rising A with B low). The
  // firmware has always counted that direction up (1x decoding before #69),
  // so the invert_encoder_* flags keep their meaning
  int64_t forwardCount_() const { return -pcnt_.getCount(); }

  int64_t count_() const { return sign_ * forwardCount_() + offset_; }

public:
  // stores the pins only, ESP32Encoder's constructor doesn't touch hardware
  // either. Global objects are constructed before the core is up
  Encoder(uint8_t signal1_pin, uint8_t signal2_pin)
      : sign_(1), offset_(0), signal1_pin_(signal1_pin),
        signal2_pin_(signal2_pin) {}

  // the PCNT unit belongs to this object
  Encoder(const Encoder &) = delete;
  Encoder &operator=(const Encoder &) = delete;

  // attaches a PCNT unit in full quadrature - should be called in main setup()
  void setup() {
    // the encoder outputs are open collector, the weak internal pull-ups keep
    // them at a defined level (the library's default is pull-down). The
    // setting is static and read by attach*()
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    pcnt_.attachFullQuad(signal1_pin_, signal2_pin_);
    pcnt_.setFilter(k_glitch_filter_apb_cycles);
  }

  // flips counting direction of encoder. The count keeps its value, only
  // later ticks count the other way
  void setInverted(bool inverted) {
    const int8_t sign = inverted ? -1 : 1;
    if (sign == sign_) {
      return;
    }
    // sign_ * raw + offset_ == sign * raw + new offset
    offset_ += 2 * sign_ * forwardCount_();
    sign_ = sign;
  }

  // the library counts in 64 bit. The wire protocol carries int32, the
  // narrowing wraps (after 2^31 counts, ~60 h at full speed) and the pi only
  // uses differences between reports, which survive the wrap
  int32_t get_tick_count() const {
    return static_cast<int32_t>(static_cast<uint32_t>(count_()));
  }
};

#endif
