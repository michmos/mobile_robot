#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include <algorithm>
#include <cmath>

class MotorControl {
private:
  // configs
  float min_duty_left_ = 0.0f;
  float min_duty_right_ = 0.0f;
  float slope_left_ = 0.0f;
  float slope_right_ = 0.0f;
  float kp_ = 0.0f;
  float ki_ = 0.0f;

  float integral_term_left_ = 0.0f;
  float integral_term_right_ = 0.0f;
  float integral_limit_ = 0.0f;

  float clamp_(float val, float min_val = -1, float max_val = 1) {
    return std::max(min_val, std::min(max_val, val));
  }

  float feedforward_(float cmdVel, float slope, float min_duty) {
    if (cmdVel == 0.0f) {
      return 0.0f;
    }
    return clamp_(copysignf(min_duty + slope * fabsf(cmdVel), cmdVel));
  }

  // ff + pi compute duty cycle
  float computeDuty_(float cmdVel, float currVel, float dt_s, bool left) {
    float errTerm = cmdVel - currVel;

    // chose parameters for left or right wheel
    float slope = (left) ? slope_left_ : slope_right_;
    float min_duty = (left) ? min_duty_left_ : min_duty_right_;
    float *integral_term =
        (left) ? &integral_term_left_ : &integral_term_right_;

    // ff + pi
    float duty =
        feedforward_(cmdVel, slope, min_duty) + kp_ * errTerm + *integral_term;

    // avoid windup by conditionally integrating only when not saturated
    bool saturated =
        (duty > 1.0f && errTerm > 0.0f) || (duty < -1.0f && errTerm < 0.0f);
    if (!saturated) {
      *integral_term += ki_ * errTerm * dt_s;

      // clamp integral term to limit
      if (*integral_term > integral_limit_) {
        *integral_term = integral_limit_;
      } else if (*integral_term < -integral_limit_) {
        *integral_term = -integral_limit_;
      }
    }
    return clamp_(duty);
  }

public:
  // setup the controller with new parameters and zeros integrals
  // all values normalized
  void configure(float min_duty_left, float min_duty_right, float slope_left,
                 float slope_right, float kp, float ki, float integral_limit) {
    min_duty_left_ = min_duty_left;
    min_duty_right_ = min_duty_right;
    slope_left_ = slope_left;
    slope_right_ = slope_right;
    kp_ = kp;
    ki_ = ki;
    integral_limit_ = integral_limit;

    // integrals should be reset when gains are changed to avoid jumps in duty
    // cycle
    reset();
  };

  // resets controller to configured state
  void reset() {
    integral_term_left_ = 0.0f;
    integral_term_right_ = 0.0f;
  }

  // get normalized duty cycle through ff and pi control
  // @return: normalized duty cycle to apply to the left wheel in [-1;1]
  float computeDutyLeft(float cmdVel, float currVel, float dt_s) {
    return computeDuty_(cmdVel, currVel, dt_s, true);
  }

  // get normalized duty cycle through ff and pi control
  // @return: normalized duty cycle to apply to the Right wheel in [-1;1]
  float computeDutyRight(float cmdVel, float currVel, float dt_s) {
    return computeDuty_(cmdVel, currVel, dt_s, false);
  }

  // get normalized duty cycle through ff control only
  // @return: normalized duty cycle to apply to the left wheel in [-1;1]
  float feedforwardLeft(float cmdVel) {
    return feedforward_(cmdVel, slope_left_, min_duty_left_);
  }

  // get normalized duty cycle through ff control only
  // @return: normalized duty cycle to apply to the right wheel in [-1;1]
  float feedforwardRight(float cmdVel) {
    return feedforward_(cmdVel, slope_right_, min_duty_right_);
  }
};

#endif
