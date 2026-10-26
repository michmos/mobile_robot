#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "Encoder.hpp"
#include "L298.hpp"
#include "Messages.hpp"
#include "MotorControl.hpp"
#include "SerialChannel.hpp"
#include "Velocity.hpp"
#include <Arduino.h>
#include <cmath>
#include <cstdint>

// tick counts of both encoders, taken at the same time
struct EncoderSample {
  int32_t left_ticks = 0;
  int32_t right_ticks = 0;
  uint32_t us = 0;
};

// @return: true if a motor command received at cmd_ms is too old to be
// followed at now_ms. uint32_t like millis() on the esp32, so the difference
// stays correct across the ~49.7 day wrap
inline bool commandStale(uint32_t now_ms, uint32_t cmd_ms,
                         uint32_t timeout_ms) {
  return now_ms - cmd_ms > timeout_ms;
}

// glues the serial channel to the motors: adopts new configurations and turns
// the latest motor command into duty cycles with the FF + PI controller. The
// peripherals are owned by the caller, this class only holds references to them
class Drive {
private:
  SerialChannel &sc_;
  L298N &motorDriver_;
  Encoder &encoderLeft_;
  Encoder &encoderRight_;
  MotorControl &motorController_;

  msgs::Config config_;

  // indicates if the motors were stopped in the last control loop iteration
  bool wasStopped_ = true;
  EncoderSample lastSample_;

public:
  Drive(SerialChannel &sc, L298N &motorDriver, Encoder &encoderLeft,
        Encoder &encoderRight, MotorControl &motorController)
      : sc_(sc), motorDriver_(motorDriver), encoderLeft_(encoderLeft),
        encoderRight_(encoderRight), motorController_(motorController) {}

  Drive(const Drive &) = delete;
  Drive &operator=(const Drive &) = delete;

  // the configuration currently applied, not initialized before the first one
  const msgs::Config &config() const { return config_; }

  // adopt a newly received configuration
  // @return: true if a new configuration was applied
  bool updateConfig() {
    const msgs::Config &received = sc_.getConfig();

    if (!received.initialized) {
      return false;
    }

    // a re-send of the handshake already applied. The id alone isn't enough:
    // a restarted pi can reuse the id the esp32 last applied, with other values
    if (config_.initialized && received.handshake_id == config_.handshake_id &&
        protocol::samePayload(received, config_)) {
      return false;
    }

    motorDriver_.brake();
    wasStopped_ = true;

    // update component configs
    motorController_.configure(received.min_duty_left, received.min_duty_right,
                               received.slope_left, received.slope_right,
                               received.kp, received.ki,
                               received.integral_limit);
    motorDriver_.setInverted1(received.invert_motor_left);
    motorDriver_.setInverted2(received.invert_motor_right);
    encoderLeft_.setInverted(received.invert_encoder_left);
    encoderRight_.setInverted(received.invert_encoder_right);

    config_ = received;

    sc_.writeln(msgs::Log("New configuration received"));
    return true;
  }

  // update the motor driver with the latest command from the serial channel
  // converted to a duty cycle using the FF + PI controller
  void updateMotor(const EncoderSample &e) {
    msgs::MotorCmd cmd = sc_.getMotorCmd();
    if (commandStale(millis(), cmd.timestamp_ms,
                     config_.watchdog_timeout_ms)) {
      motorDriver_.brake();
      motorController_.reset();
      wasStopped_ = true;
      sc_.writeln(msgs::Log("No new motor command received, stopping motors"));
      return;
    }

    // ---- get new duty ----
    float dutyLeft, dutyRight;
    if (wasStopped_) {
      // ff without pi to avoid calculating velocity from stale data
      dutyLeft = motorController_.feedforwardLeft(cmd.left_vel_cmd);
      dutyRight = motorController_.feedforwardRight(cmd.right_vel_cmd);

    } else {
      // get current velocity
      const uint32_t dt_us = e.us - lastSample_.us;
      float velLeft = getVelocity(e.left_ticks, lastSample_.left_ticks, dt_us,
                                  config_.ticks_per_wheel_rev);
      float velRight = getVelocity(e.right_ticks, lastSample_.right_ticks,
                                   dt_us, config_.ticks_per_wheel_rev);
      float dt_s = dt_us * 1e-6f;

      dutyLeft = motorController_.computeDutyLeft(cmd.left_vel_cmd, velLeft,
                                                  dt_s);
      dutyRight = motorController_.computeDutyRight(cmd.right_vel_cmd,
                                                    velRight, dt_s);
    }
    wasStopped_ = false;
    lastSample_ = e;

    // ---- update duty on motor driver ----
    bool motorError = false;
    if (cmd.left_vel_cmd == 0.0f) {
      motorDriver_.brake1();
    } else {
      motorDriver_.updateDir1((dutyLeft >= 0.0f) ? FORWARDS : BACKWARDS);
      motorError |= (motorDriver_.setDuty1(fabsf(dutyLeft)) == -1);
    }
    if (cmd.right_vel_cmd == 0.0f) {
      motorDriver_.brake2();
    } else {
      motorDriver_.updateDir2((dutyRight >= 0.0f) ? FORWARDS : BACKWARDS);
      motorError |= (motorDriver_.setDuty2(fabsf(dutyRight)) == -1);
    }
    if (motorError) {
      sc_.writeln(msgs::Log("Could not update motor duty cycles"));
    }
  }
};

#endif
