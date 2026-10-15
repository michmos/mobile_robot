// host tests for the firmware's quadrature Encoder wrapper,
// compiled against test/arduino_stub/Arduino.h
#include "inc/Encoder.hpp"

#include <gtest/gtest.h>

#include <type_traits>

// the encoder owns its pins and its isr gets `this`, a copy would count into
// the wrong object
static_assert(!std::is_copy_constructible_v<Encoder>);
static_assert(!std::is_copy_assignable_v<Encoder>);

namespace {

// the pins esp32_firmware.ino uses for encoder 1
constexpr uint8_t k_a = 18;
constexpr uint8_t k_b = 19;

class EncoderTest : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }

  // one rising edge on A, with B at the given level
  void edge(int b_level) {
    arduino_stub::setDigitalRead(k_b, b_level);
    ASSERT_TRUE(arduino_stub::fireInterrupt(k_a));
  }

  Encoder encoder_{k_a, k_b};
};

TEST_F(EncoderTest, ConstructorDoesNotTouchHardware) {
  // global objects are constructed before the core is up, all hardware access
  // has to wait for setup()
  Encoder encoder(k_a, k_b);

  EXPECT_EQ(arduino_stub::pinModeOf(k_a), -1);
  EXPECT_EQ(arduino_stub::pinModeOf(k_b), -1);
  EXPECT_FALSE(arduino_stub::hasInterrupt(k_a));
}

TEST_F(EncoderTest, StartsAtZero) {
  EXPECT_EQ(encoder_.get_tick_count(), 0);
  encoder_.setup();
  EXPECT_EQ(encoder_.get_tick_count(), 0);
}

TEST_F(EncoderTest, SetupConfiguresPinsWithPullup) {
  // keeps the inputs at a defined level while the encoder doesn't drive them
  encoder_.setup();

  EXPECT_EQ(arduino_stub::pinModeOf(k_a), INPUT_PULLUP);
  EXPECT_EQ(arduino_stub::pinModeOf(k_b), INPUT_PULLUP);
}

TEST_F(EncoderTest, SetupAttachesRisingEdgeInterruptOnAOnly) {
  // one tick per period of A (x1 decoding), B is only sampled
  encoder_.setup();

  EXPECT_TRUE(arduino_stub::hasInterrupt(k_a));
  EXPECT_EQ(arduino_stub::interruptMode(k_a), RISING);
  EXPECT_FALSE(arduino_stub::hasInterrupt(k_b));
}

TEST_F(EncoderTest, EdgeWithBLowCountsUp) {
  encoder_.setup();

  edge(LOW);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
  edge(LOW);
  edge(LOW);
  EXPECT_EQ(encoder_.get_tick_count(), 3);
}

TEST_F(EncoderTest, EdgeWithBHighCountsDown) {
  encoder_.setup();

  edge(HIGH);
  EXPECT_EQ(encoder_.get_tick_count(), -1);
  edge(HIGH);
  EXPECT_EQ(encoder_.get_tick_count(), -2);
}

TEST_F(EncoderTest, DirectionChangeCountsBackThroughZero) {
  // the wheel turning back and forth must not drift
  encoder_.setup();

  edge(LOW);
  edge(LOW);
  edge(HIGH);
  edge(HIGH);
  EXPECT_EQ(encoder_.get_tick_count(), 0);
  edge(HIGH);
  EXPECT_EQ(encoder_.get_tick_count(), -1);
}

TEST_F(EncoderTest, InvertedFlipsCountingDirection) {
  // an encoder mounted mirrored sees the phases the other way round, the
  // config's invert flag compensates
  encoder_.setInverted(true);
  encoder_.setup();

  edge(LOW);
  EXPECT_EQ(encoder_.get_tick_count(), -1);
  edge(HIGH);
  edge(HIGH);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
}

TEST_F(EncoderTest, InvertedCanBeCleared) {
  // a new config with invert = false has to undo an earlier invert = true
  encoder_.setup();
  encoder_.setInverted(true);
  encoder_.setInverted(false);

  edge(LOW);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
}

TEST_F(EncoderTest, InvertingKeepsTheCount) {
  // a config applied at runtime only changes how future ticks count, the
  // pi side works with tick deltas so the count must not jump
  encoder_.setup();
  edge(LOW);
  edge(LOW);

  encoder_.setInverted(true);
  EXPECT_EQ(encoder_.get_tick_count(), 2);

  edge(LOW);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
}

TEST_F(EncoderTest, ReadingDoesNotResetTheCount) {
  // the firmware has no reset, it reports the running count and the pi
  // computes deltas, so reading must be side effect free
  encoder_.setup();
  edge(LOW);

  EXPECT_EQ(encoder_.get_tick_count(), 1);
  EXPECT_EQ(encoder_.get_tick_count(), 1);
}

TEST_F(EncoderTest, EncodersCountIndependently) {
  // both wheels' isrs share the same static function, `this` picks the
  // encoder
  constexpr uint8_t k_a2 = 22;
  constexpr uint8_t k_b2 = 23;
  Encoder encoder2(k_a2, k_b2);
  encoder_.setup();
  encoder2.setup();

  edge(LOW);
  arduino_stub::setDigitalRead(k_b2, HIGH);
  ASSERT_TRUE(arduino_stub::fireInterrupt(k_a2));
  ASSERT_TRUE(arduino_stub::fireInterrupt(k_a2));

  EXPECT_EQ(encoder_.get_tick_count(), 1);
  EXPECT_EQ(encoder2.get_tick_count(), -2);
}

} // namespace
