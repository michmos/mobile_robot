#ifndef MOBILE_ROBOT_HARDWARE_HANDSHAKE_HPP
#define MOBILE_ROBOT_HARDWARE_HANDSHAKE_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

namespace mobile_robot_hardware {

// pi side state of the configuration handshake with the esp32
//
// One handshake is identified by the id the pi puts into the config line.
// Re-sends keep the id, the esp32 echoes it in ACK/NACK, so replies that
// belong to an older handshake can be told apart and ignored.
// Ids are strictly increasing within one plugin run, so a reply can always be
// matched to its handshake. Across runs (pi reboot, 32 bit ms wrap) an id can
// come up again; the esp32 then still applies the config because it only
// skips a re-send if both id and payload match what it runs.
// No I/O and no clock of its own: the caller passes the time in
class Handshake {
public:
  using Clock = std::chrono::steady_clock;

  enum class State {
    IDLE,       // nothing started yet
    PENDING,    // config is (re-)sent until the esp32 replies
    CONFIGURED, // matching ACK received
    FAILED,     // deadline passed without a matching ACK
  };

  Handshake(std::chrono::milliseconds retryInterval,
            std::chrono::milliseconds timeout)
      : retryInterval_(retryInterval), timeout_(timeout) {}

  // begin a new handshake, the config is due immediately
  // @return: the id to put into the config line, never 0
  uint32_t start(Clock::time_point now) {
    // based on the time rather than a counter, so a restarted plugin rarely
    // reuses the id the esp32 last applied
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch())
                        .count();
    id_ = static_cast<uint32_t>(ms);
    if (id_ <= previousId_) {
      // same millisecond or the 32 bit ms wrapped: keep counting up instead
      id_ = previousId_ + 1;
    }
    // 0 is reserved as "no handshake" on the wire, only reached when the ids
    // wrap around UINT32_MAX
    if (id_ == 0) {
      id_ = 1;
    }
    previousId_ = id_;
    state_ = State::PENDING;
    startedAt_ = now;
    nextSendAt_ = now;
    failureReason_.clear();
    lastNackField_.clear();
    return id_;
  }

  // @return: true if the config should be sent now. Moves to FAILED once the
  //          deadline has passed instead
  bool dueForSend(Clock::time_point now) {
    if (state_ != State::PENDING) {
      return false;
    }
    if (now - startedAt_ >= timeout_) {
      std::string reason =
          "timed out waiting for the esp32 to acknowledge the configuration";
      if (!lastNackField_.empty()) {
        reason += ", last NACK named field '" + lastNackField_ + "'";
      }
      fail_(reason);
      return false;
    }
    if (now < nextSendAt_) {
      return false;
    }
    nextSendAt_ = now + retryInterval_;
    return true;
  }

  // the esp32 asks for a configuration. A handshake that is already running
  // is left alone, its retry interval already re-sends the config
  // @return: true if the SETUP started a new handshake
  bool onSetup(Clock::time_point now) {
    if (state_ == State::PENDING) {
      return false;
    }
    start(now);
    return true;
  }

  // the esp32 acknowledged handshake `id`. Replies to other handshakes and
  // replies outside PENDING are ignored
  void onAck(uint32_t id) {
    if (state_ == State::PENDING && id == id_) {
      state_ = State::CONFIGURED;
    }
  }

  // the esp32 rejected handshake `id` because of `field`. The pi validates
  // with the same check and serializes losslessly, so a NACK means the line
  // got corrupted on the way: the handshake stays PENDING and the config is
  // re-sent on the retry interval until an ACK or the deadline. Replies to
  // other handshakes and replies outside PENDING are ignored
  // @return: true if the NACK belonged to the running handshake
  bool onNack(uint32_t id, const std::string &field) {
    if (state_ != State::PENDING || id != id_) {
      return false;
    }
    lastNackField_ = field;
    return true;
  }

  // when dueForSend() next has something to do: a re-send or the deadline.
  // Only meaningful while PENDING
  Clock::time_point nextEventAt() const {
    return std::min(nextSendAt_, startedAt_ + timeout_);
  }

  State state() const { return state_; }
  uint32_t id() const { return id_; }
  const std::string &failureReason() const { return failureReason_; }

private:
  void fail_(const std::string &reason) {
    state_ = State::FAILED;
    failureReason_ = reason;
  }

  const std::chrono::milliseconds retryInterval_;
  const std::chrono::milliseconds timeout_;

  State state_ = State::IDLE;
  uint32_t id_ = 0;
  uint32_t previousId_ = 0;
  Clock::time_point startedAt_;
  Clock::time_point nextSendAt_;
  std::string failureReason_;
  std::string lastNackField_;
};

} // namespace mobile_robot_hardware

#endif
