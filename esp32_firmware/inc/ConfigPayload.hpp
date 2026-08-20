#ifndef CONFIGPAYLOAD_HPP
#define CONFIGPAYLOAD_HPP

#include <cstdint>

// Payload of the Config ("C,...") message and its range check, see
// Messages.hpp for the wire format.
//
// Shared by both ends of the serial link: pi and esp32
// the check needs to run on the pi to ensure sanity of the values
// and on the esp32 to make sure the values arrived without any noise
// Keeping the check in one place means the pi cannot accept a
// configuration the esp32 would reject.

namespace protocol {

///////////////////////////////////////////////////////////////////////
// LIMITS
///////////////////////////////////////////////////////////////////////

// a duty fraction
constexpr float k_gain_min = 0.0f; // inclusive

// feedforward slope, duty fraction per (rad/s)
constexpr float k_slope_min = 0.0f; // exclusive

// deadband compensation, a duty fraction
constexpr float k_min_duty_min = 0.0f; // inclusive
constexpr float k_min_duty_max = 1.0f; // exclusive

// divides the tick delta in the tick -> rad conversion
constexpr float k_ticks_per_wheel_rev_min = 0.0f; // exclusive

// timing. The wire format carries these as uint16_t, and the periods are
// derived as 1000000 / rate_hz, so zero is not allowed
constexpr uint16_t k_timing_min = 1;
constexpr uint16_t k_timing_max = UINT16_MAX;

struct ConfigPayload {
  // PI gains - shared by both wheels
  float kp = 0.0f;
  float ki = 0.0f;
  float integral_limit = 0.0f; // anti-windup clamp on |integral term|

  // feedforward - per wheel, since stiction and torque differ normally
  float slope_left = 0.1f;
  float slope_right = 0.1f;
  float min_duty_left = 0.0f;
  float min_duty_right = 0.0f;

  // counted encoder edges per wheel revolution
  float ticks_per_wheel_rev = 480.0f;

  uint16_t watchdog_timeout_ms = 200; // stop motors if no MotorCmd within this
  uint16_t control_rate_hz = 200;     // PI update rate
  uint16_t report_rate_hz = 50;       // EncoderData publish rate

  // sign conventions - the two motors are mounted mirrored, so "forwards" is
  // opposite rotation on each side
  bool invert_left = false;
  bool invert_right = false;
};

// fields of ConfigPayload, ordered as they appear on the wire
enum e_config_field {
  CONFIG_OK = 0,
  CONFIG_KP,
  CONFIG_KI,
  CONFIG_INTEGRAL_LIMIT,
  CONFIG_SLOPE_LEFT,
  CONFIG_SLOPE_RIGHT,
  CONFIG_MIN_DUTY_LEFT,
  CONFIG_MIN_DUTY_RIGHT,
  CONFIG_TICKS_PER_WHEEL_REV,
  CONFIG_WATCHDOG_TIMEOUT_MS,
  CONFIG_CONTROL_RATE_HZ,
  CONFIG_REPORT_RATE_HZ,
};

// name of a field, sent as the detail of a NACK event and logged by the pi
inline const char *configFieldName(e_config_field field) {
  switch (field) {
  case CONFIG_OK:
    return "ok";
  case CONFIG_KP:
    return "kp";
  case CONFIG_KI:
    return "ki";
  case CONFIG_INTEGRAL_LIMIT:
    return "integral_limit";
  case CONFIG_SLOPE_LEFT:
    return "slope_left";
  case CONFIG_SLOPE_RIGHT:
    return "slope_right";
  case CONFIG_MIN_DUTY_LEFT:
    return "min_duty_left";
  case CONFIG_MIN_DUTY_RIGHT:
    return "min_duty_right";
  case CONFIG_TICKS_PER_WHEEL_REV:
    return "ticks_per_wheel_rev";
  case CONFIG_WATCHDOG_TIMEOUT_MS:
    return "watchdog_timeout_ms";
  case CONFIG_CONTROL_RATE_HZ:
    return "control_rate_hz";
  case CONFIG_REPORT_RATE_HZ:
    return "report_rate_hz";
  }
  return "unknown";
}

// range check of a configuration
// @return CONFIG_OK, or the first field that is out of range
inline e_config_field validateConfig(const ConfigPayload &c) {
  if (!(c.kp >= k_gain_min)) {
    return CONFIG_KP;
  }
  if (!(c.ki >= k_gain_min)) {
    return CONFIG_KI;
  }
  if (!(c.integral_limit >= k_gain_min)) {
    return CONFIG_INTEGRAL_LIMIT;
  }
  if (!(c.slope_left > k_slope_min)) {
    return CONFIG_SLOPE_LEFT;
  }
  if (!(c.slope_right > k_slope_min)) {
    return CONFIG_SLOPE_RIGHT;
  }
  if (!(c.min_duty_left >= k_min_duty_min &&
        c.min_duty_left < k_min_duty_max)) {
    return CONFIG_MIN_DUTY_LEFT;
  }
  if (!(c.min_duty_right >= k_min_duty_min &&
        c.min_duty_right < k_min_duty_max)) {
    return CONFIG_MIN_DUTY_RIGHT;
  }
  if (!(c.ticks_per_wheel_rev > k_ticks_per_wheel_rev_min)) {
    return CONFIG_TICKS_PER_WHEEL_REV;
  }
  if (c.watchdog_timeout_ms < k_timing_min) {
    return CONFIG_WATCHDOG_TIMEOUT_MS;
  }
  if (c.control_rate_hz < k_timing_min) {
    return CONFIG_CONTROL_RATE_HZ;
  }
  // a report faster than the control loop would send duplicate samples
  if (c.report_rate_hz < k_timing_min || c.report_rate_hz > c.control_rate_hz) {
    return CONFIG_REPORT_RATE_HZ;
  }
  return CONFIG_OK;
}

} // namespace protocol

#endif
