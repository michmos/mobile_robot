// host tests for the firmware's quadrature Encoder wrapper around the
// ESP32Encoder library, compiled against test/arduino_stub/ (Arduino.h and
// the library fake ESP32Encoder.h)
#include "inc/Encoder.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <type_traits>

// the encoder owns its PCNT unit, a copy would share or re-attach it
static_assert(!std::is_copy_constructible_v<Encoder>);
static_assert(!std::is_copy_assignable_v<Encoder>);

namespace {

// the pins esp32_firmware.ino uses for the right encoder
constexpr uint8_t k_a = 18;
constexpr uint8_t k_b = 19;

class EncoderTest : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }

  // the library count of encoder_. The library counts down while A leads B,
  // the direction the firmware reports as positive
  static void libraryCount(int64_t count) {
    ASSERT_TRUE(arduino_stub::setEncoderCount(k_a, count));
  }

  Encoder encoder_{k_a, k_b};
};

TEST_F(EncoderTest, ConstructorDoesNotTouchHardware) {
  // global objects are constructed before the core is up, all hardware access
  // has to wait for setup()
  Encoder encoder(k_a, k_b);

  EXPECT_FALSE(arduino_stub::encoderAttached(k_a));
  EXPECT_EQ(arduino_stub::pinModeOf(k_a), -1);
  EXPECT_EQ(arduino_stub::pinModeOf(k_b), -1);
  EXPECT_FALSE(arduino_stub::hasInterrupt(k_a));
}

TEST_F(EncoderTest, CountIsZeroBeforeSetup) {
  EXPECT_EQ(encoder_.get_tick_count(), 0);
}

TEST_F(EncoderTest, SetupAttachesFullQuadratureOnBothPins) {
  // 4x decoding: a glitch on one channel counts +1 and -1 and cancels (#69)
  encoder_.setup();

  ASSERT_TRUE(arduino_stub::encoderAttached(k_a));
  const auto unit = arduino_stub::encoderUnit(k_a);
  EXPECT_EQ(unit.a_pin, k_a);
  EXPECT_EQ(unit.b_pin, k_b);
  EXPECT_EQ(unit.type, encType::full);
  EXPECT_EQ(encoder_.get_tick_count(), 0);
}

TEST_F(EncoderTest, SetupUsesInternalPullUps) {
  // the outputs are open collector. The library defaults to pull-down, which
  // would hold both lines low
  encoder_.setup();

  EXPECT_EQ(arduino_stub::encoderUnit(k_a).pull, puType::up);
}

TEST_F(EncoderTest, SetupSetsTheGlitchFilter) {
  // 400 APB cycles = 5 us, the library's attach() default is 250
  encoder_.setup();

  EXPECT_EQ(arduino_stub::encoderUnit(k_a).filter, 400);
  EXPECT_EQ(Encoder::k_glitch_filter_apb_cycles, 400);
}

TEST_F(EncoderTest, GlitchFilterStaysWithinTheLibraryRange) {
  // larger values are clamped to 1023 by the library, 0 turns it off
  EXPECT_GT(Encoder::k_glitch_filter_apb_cycles, 0);
  EXPECT_LE(Encoder::k_glitch_filter_apb_cycles, 1023);
}

TEST_F(EncoderTest, ALeadingBCountsUp) {
  // one full A/B cycle with A leading is -4 in the library, +4 here, the
  // direction the 1x firmware counted up, so invert_encoder_* keep their
  // meaning
  encoder_.setup();

  libraryCount(-4);
  EXPECT_EQ(encoder_.get_tick_count(), 4);
}

TEST_F(EncoderTest, BLeadingACountsDown) {
  encoder_.setup();

  libraryCount(4);
  EXPECT_EQ(encoder_.get_tick_count(), -4);
}

TEST_F(EncoderTest, CountPassesThroughUnscaled) {
  // every edge is one tick, ticks_per_wheel_rev is in the same unit
  encoder_.setup();

  libraryCount(-1964);
  EXPECT_EQ(encoder_.get_tick_count(), 1964);
  libraryCount(0);
  EXPECT_EQ(encoder_.get_tick_count(), 0);
  libraryCount(1);
  EXPECT_EQ(encoder_.get_tick_count(), -1);
}

TEST_F(EncoderTest, InvertedFlipsCountingDirection) {
  // an encoder mounted mirrored sees the phases the other way round, the
  // config's invert flag compensates
  encoder_.setInverted(true);
  encoder_.setup();

  libraryCount(-4);
  EXPECT_EQ(encoder_.get_tick_count(), -4);
  libraryCount(3);
  EXPECT_EQ(encoder_.get_tick_count(), 3);
}

TEST_F(EncoderTest, InvertedCanBeCleared) {
  // a new config with invert = false has to undo an earlier invert = true
  encoder_.setup();
  encoder_.setInverted(true);
  encoder_.setInverted(false);

  libraryCount(-1);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
}

TEST_F(EncoderTest, SettingTheSameInversionAgainChangesNothing) {
  // every handshake re-applies the config, mostly with the same flags
  encoder_.setup();
  libraryCount(-10);
  encoder_.setInverted(false);
  EXPECT_EQ(encoder_.get_tick_count(), 10);

  encoder_.setInverted(true);
  encoder_.setInverted(true);
  libraryCount(-12);
  EXPECT_EQ(encoder_.get_tick_count(), 8);
}

TEST_F(EncoderTest, InvertingKeepsTheCount) {
  // a config applied at runtime only changes how future ticks count, the
  // pi side works with tick deltas so the count must not jump
  encoder_.setup();
  libraryCount(-2);

  encoder_.setInverted(true);
  EXPECT_EQ(encoder_.get_tick_count(), 2);

  libraryCount(-3);
  EXPECT_EQ(encoder_.get_tick_count(), 1);

  encoder_.setInverted(false);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
  libraryCount(-5);
  EXPECT_EQ(encoder_.get_tick_count(), 3);
}

TEST_F(EncoderTest, ReadingDoesNotResetTheCount) {
  // the firmware has no reset, it reports the running count and the pi
  // computes deltas, so reading must be side effect free
  encoder_.setup();
  libraryCount(-1);

  EXPECT_EQ(encoder_.get_tick_count(), 1);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
}

TEST_F(EncoderTest, Int32LimitsPassThrough) {
  encoder_.setup();

  libraryCount(-static_cast<int64_t>(std::numeric_limits<int32_t>::max()));
  EXPECT_EQ(encoder_.get_tick_count(), std::numeric_limits<int32_t>::max());
  libraryCount(-static_cast<int64_t>(std::numeric_limits<int32_t>::min()));
  EXPECT_EQ(encoder_.get_tick_count(), std::numeric_limits<int32_t>::min());
}

TEST_F(EncoderTest, CountBeyondInt32Wraps) {
  // the library counts in 64 bit, the wire protocol carries int32. The
  // narrowing wraps modulo 2^32, so differences between two reports stay
  // right across the wrap
  encoder_.setup();
  const int64_t max = std::numeric_limits<int32_t>::max();

  libraryCount(-max);
  const int32_t before = encoder_.get_tick_count();
  libraryCount(-(max + 5));
  const int32_t after = encoder_.get_tick_count();

  EXPECT_EQ(after, std::numeric_limits<int32_t>::min() + 4);
  EXPECT_EQ(static_cast<int32_t>(static_cast<uint32_t>(after) -
                                 static_cast<uint32_t>(before)),
            5);
}

TEST_F(EncoderTest, EncodersCountIndependently) {
  // both wheels get their own PCNT unit
  constexpr uint8_t k_a2 = 22;
  constexpr uint8_t k_b2 = 23;
  Encoder encoder2(k_a2, k_b2);
  encoder_.setup();
  encoder2.setup();

  libraryCount(-1);
  ASSERT_TRUE(arduino_stub::setEncoderCount(k_a2, 2));

  EXPECT_EQ(encoder_.get_tick_count(), 1);
  EXPECT_EQ(encoder2.get_tick_count(), -2);
  EXPECT_EQ(arduino_stub::encoderUnit(k_a2).b_pin, k_b2);
}

} // namespace
