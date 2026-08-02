#ifndef L298_HPP
#define L298_HPP

#include <Arduino.h>
#include <cstdint>

enum e_direction { BACKWARDS = -1, FORWARDS = 1 };

class L298N {
private:
  // ESP32 pins
  uint8_t _ENA_Pin;
  uint8_t _ENB_Pin;
  uint8_t _IN1_Pin;
  uint8_t _IN2_Pin;
  uint8_t _IN3_Pin;
  uint8_t _IN4_Pin;

public:
  L298N(uint8_t ENA_Pin, uint8_t IN1_Pin, uint8_t IN2_Pin, uint8_t ENB_Pin,
        uint8_t IN3_Pin, uint8_t IN4_Pin)
      : _ENA_Pin(ENA_Pin), _ENB_Pin(ENB_Pin), _IN1_Pin(IN1_Pin),
        _IN2_Pin(IN2_Pin), _IN3_Pin(IN3_Pin), _IN4_Pin(IN4_Pin) {}

  L298N(const L298N &toCopy) = delete;
  L298N &operator=(const L298N &toAssign) = delete;

  void setup(uint8_t pwm_res) {
    // motor setup
    pinMode(_ENA_Pin, OUTPUT);
    pinMode(_IN1_Pin, OUTPUT);
    pinMode(_IN2_Pin, OUTPUT);
    pinMode(_ENB_Pin, OUTPUT);
    pinMode(_IN3_Pin, OUTPUT);
    pinMode(_IN4_Pin, OUTPUT);
    // turn motor off
    digitalWrite(_ENA_Pin, LOW);
    digitalWrite(_IN1_Pin, LOW);
    digitalWrite(_IN2_Pin, LOW);
    digitalWrite(_ENB_Pin, LOW);
    digitalWrite(_IN3_Pin, LOW);
    digitalWrite(_IN4_Pin, LOW);
    // set up one pwm channel for each motor
    ledcSetup(1, 1000, pwm_res);
    ledcAttachPin(_ENA_Pin, 1);
    ledcSetup(2, 1000, pwm_res);
    ledcAttachPin(_ENB_Pin, 2);
  }
  void updateDirection1(e_direction dir = FORWARDS) {
    if (dir == BACKWARDS) {
      digitalWrite(_IN1_Pin, HIGH);
      digitalWrite(_IN2_Pin, LOW);
    } else {
      digitalWrite(_IN1_Pin, LOW);
      digitalWrite(_IN2_Pin, HIGH);
    }
  }
  void updateDirection2(e_direction dir = FORWARDS) {
    if (dir == BACKWARDS) {
      digitalWrite(_IN3_Pin, HIGH);
      digitalWrite(_IN4_Pin, LOW);
    } else {
      digitalWrite(_IN3_Pin, LOW);
      digitalWrite(_IN4_Pin, HIGH);
    }
  }
  void updateSpeed(uint16_t duty_cycle1, uint16_t duty_cycle2) {
    ledcWrite(1, duty_cycle1);
    ledcWrite(2, duty_cycle2);
  }
};

#endif
