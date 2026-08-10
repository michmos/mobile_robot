#ifndef L298_HPP
#define L298_HPP

#include <Arduino.h>
#include <cmath>
#include <cstdint>

enum e_direction { BACKWARDS = -1, FORWARDS = 1 };

class L298N {
private:
  static const uint32_t k_pwm_freq_hz_ = 1000;
  static const uint8_t k_channel1_ = 1;
  static const uint8_t k_channel2_ = 2;

  // ESP32 pins
  uint8_t ENA_Pin_;
  uint8_t ENB_Pin_;
  uint8_t IN1_Pin_;
  uint8_t IN2_Pin_;
  uint8_t IN3_Pin_;
  uint8_t IN4_Pin_;

  uint8_t pwm_res_;
  uint32_t duty_max_;
  bool inverted1_;
  bool inverted2_;

public:
  L298N(uint8_t ENA_Pin, uint8_t IN1_Pin, uint8_t IN2_Pin, uint8_t ENB_Pin,
        uint8_t IN3_Pin, uint8_t IN4_Pin, uint8_t pwm_res)
      : ENA_Pin_(ENA_Pin), ENB_Pin_(ENB_Pin), IN1_Pin_(IN1_Pin),
        IN2_Pin_(IN2_Pin), IN3_Pin_(IN3_Pin), IN4_Pin_(IN4_Pin),
        pwm_res_(pwm_res), duty_max_((1u << pwm_res) - 1u), inverted1_(false),
        inverted2_(false) {}

  L298N(const L298N &toCopy) = delete;
  L298N &operator=(const L298N &toAssign) = delete;

  void setup() {
    // motor setup
    pinMode(ENA_Pin_, OUTPUT);
    pinMode(IN1_Pin_, OUTPUT);
    pinMode(IN2_Pin_, OUTPUT);
    pinMode(ENB_Pin_, OUTPUT);
    pinMode(IN3_Pin_, OUTPUT);
    pinMode(IN4_Pin_, OUTPUT);
    // turn motor off
    digitalWrite(ENA_Pin_, LOW);
    digitalWrite(ENB_Pin_, LOW);
    stop();

    // set up one pwm channel for each motor
    ledcSetup(k_channel1_, k_pwm_freq_hz_, pwm_res_);
    ledcAttachPin(ENA_Pin_, k_channel1_);
    ledcSetup(k_channel2_, k_pwm_freq_hz_, pwm_res_);
    ledcAttachPin(ENB_Pin_, k_channel2_);
  }

  void stop() {
    digitalWrite(IN1_Pin_, LOW);
    digitalWrite(IN2_Pin_, LOW);
    digitalWrite(IN3_Pin_, LOW);
    digitalWrite(IN4_Pin_, LOW);
  }

  // move both motors in forward direction
  void start() {
    updateDir1(FORWARDS);
    updateDir2(FORWARDS);
  }

  // drive both motors from normalized duty cycles
  // @param duty_cycle1: normalized duty cycle for motor 1 in [0; 1]
  // @param duty_cycle2: normalized duty cycle for motor 2 in [0; 1]
  int updateDuty(float duty_cycle1, float duty_cycle2) {
    if (duty_cycle1 < 0 || duty_cycle1 > 1 || duty_cycle2 < 0 ||
        duty_cycle2 > 1) {
      return -1; // ignore invalid duty cycles
    }

    ledcWrite(k_channel1_, duty_cycle1 * duty_max_ + 0.5f);
    ledcWrite(k_channel2_, duty_cycle2 * duty_max_ + 0.5f);
    return 0;
  }

  // invert meaing of FORWARDS and BACKWARDS
  void setInverted1(bool inverted) { inverted1_ = inverted; }
  // invert meaing of FORWARDS and BACKWARDS
  void setInverted2(bool inverted) { inverted2_ = inverted; }

  void updateDir1(e_direction dir = FORWARDS) {
    dir = inverted1_ ? (dir == FORWARDS ? BACKWARDS : FORWARDS) : dir;
    if (dir == FORWARDS) {
      digitalWrite(IN1_Pin_, HIGH);
      digitalWrite(IN2_Pin_, LOW);
    } else {
      digitalWrite(IN1_Pin_, LOW);
      digitalWrite(IN2_Pin_, HIGH);
    }
  }
  void updateDir2(e_direction dir = FORWARDS) {
    dir = inverted2_ ? (dir == FORWARDS ? BACKWARDS : FORWARDS) : dir;
    if (dir == FORWARDS) {
      digitalWrite(IN3_Pin_, HIGH);
      digitalWrite(IN4_Pin_, LOW);
    } else {
      digitalWrite(IN3_Pin_, LOW);
      digitalWrite(IN4_Pin_, HIGH);
    }
  }
};

#endif
