#ifndef VELOCITY_HPP
#define VELOCITY_HPP

#include <cmath>
#include <cstdint>

// converts encoder ticks to the wheel angle in radians
// @param ticks_per_wheel_rev: counted encoder edges per wheel revolution
inline float ticksToRad(int32_t ticks, float ticks_per_wheel_rev) {
  return (ticks * 2.0f * M_PI) / ticks_per_wheel_rev;
}

// returns the velocity in radians per second
// @param dt_us: time between the two tick counts in us
// @param ticks_per_wheel_rev: counted encoder edges per wheel revolution
inline float getVelocity(int32_t ticksNow, int32_t ticksLast, uint32_t dt_us,
                         float ticks_per_wheel_rev) {
  return ticksToRad(ticksNow - ticksLast, ticks_per_wheel_rev) /
         (dt_us * 1e-6f);
}

#endif
