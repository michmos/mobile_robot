#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include <cstdint>

// periodic scheduling on the free running micros() clock. Timestamps are
// uint32_t like micros() on the esp32, which wraps every ~71.6 min

// returns true when next has been reached and updates next to the next time to
// run, otherwise just returns false.
// The comparison is on the signed difference, so it keeps working across the
// clock wrap as long as next is less than 2^31 us away from now.
// After a stall of more than one period the missed runs are dropped: it fires
// once and the schedule restarts one period after now
// @param now: current time in us
// @param next: time of the next run in us, advanced when due
// @param period_us: time between two runs in us
inline bool due(uint32_t now, uint32_t &next, uint32_t period_us) {
  if ((int32_t)(now - next) < 0) {
    return false;
  }

  // update next by period_us and reset it in case more than one period has passed
  next = (now - next > period_us) ? now + period_us : next + period_us;
  return true;
}

#endif
