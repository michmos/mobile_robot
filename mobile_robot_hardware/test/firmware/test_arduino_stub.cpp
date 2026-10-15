// checks the Arduino core fake itself, so the firmware tests built on it
// can trust what it records. Including the peripheral headers also makes sure
// the stub covers everything they use
#include "inc/Encoder.hpp"
#include "inc/L298.hpp"
#include "inc/MotorControl.hpp"
#include "inc/SerialChannel.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

class ArduinoStubTest : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }
};

int g_isrCalls = 0;
void countIsr() { ++g_isrCalls; }
void countIsrArg(void *arg) { ++*static_cast<int *>(arg); }

TEST_F(ArduinoStubTest, ClockIsControlledByTheTest) {
  EXPECT_EQ(micros(), 0UL);
  arduino_stub::setMicros(1500);
  EXPECT_EQ(micros(), 1500UL);
  // millis() is derived from the same clock, like on the real core
  EXPECT_EQ(millis(), 1UL);

  arduino_stub::setMillis(20);
  arduino_stub::advanceMicros(999);
  EXPECT_EQ(millis(), 20UL);
  arduino_stub::advanceMillis(5);
  EXPECT_EQ(micros(), 25999UL);
}

TEST_F(ArduinoStubTest, ClockWrapsAt32BitLikeTheEsp32) {
  // micros() wraps after ~71.6 min on the esp32, the firmware has to cope
  arduino_stub::setMicros(0xFFFFFFFFULL);
  EXPECT_EQ(micros(), 0xFFFFFFFFUL);
  arduino_stub::advanceMicros(1);
  EXPECT_EQ(micros(), 0UL);
  arduino_stub::advanceMicros(10);
  EXPECT_EQ(micros(), 10UL);

  // millis() wraps on its own after ~49.7 days, not together with micros()
  arduino_stub::setMillis(0xFFFFFFFFULL);
  EXPECT_EQ(millis(), 0xFFFFFFFFUL);
  arduino_stub::advanceMillis(1);
  EXPECT_EQ(millis(), 0UL);
}

TEST_F(ArduinoStubTest, DelayAdvancesTheClockWithoutBlocking) {
  delay(10);
  EXPECT_EQ(millis(), 10UL);
  delayMicroseconds(5);
  EXPECT_EQ(micros(), 10005UL);
}

TEST_F(ArduinoStubTest, SerialReadsQueuedInput) {
  Serial.begin(115200);
  EXPECT_EQ(Serial.baud(), 115200UL);
  EXPECT_EQ(Serial.available(), 0);

  Serial.pushInput("abc");
  EXPECT_EQ(Serial.available(), 3);

  // asking for more than is queued returns what is there
  char buf[8] = {};
  EXPECT_EQ(Serial.readBytes(buf, sizeof(buf)), 3u);
  EXPECT_EQ(std::string(buf, 3), "abc");
  EXPECT_EQ(Serial.available(), 0);
  EXPECT_EQ(Serial.read(), -1);
}

TEST_F(ArduinoStubTest, SerialCapturesOutput) {
  Serial.write("E,1");
  Serial.write(static_cast<uint8_t>(','));
  Serial.print(-2);
  Serial.println(3UL);
  EXPECT_EQ(Serial.output(), "E,1,-23\r\n");

  Serial.clearOutput();
  EXPECT_EQ(Serial.output(), "");
}

TEST_F(ArduinoStubTest, GpioRecordsLastValuePerPin) {
  EXPECT_EQ(arduino_stub::pinModeOf(4), -1);
  EXPECT_EQ(arduino_stub::digitalWritten(4), -1);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  digitalWrite(4, LOW);
  EXPECT_EQ(arduino_stub::pinModeOf(4), OUTPUT);
  EXPECT_EQ(arduino_stub::digitalWritten(4), LOW);

  EXPECT_EQ(digitalRead(5), LOW);
  arduino_stub::setDigitalRead(5, HIGH);
  EXPECT_EQ(digitalRead(5), HIGH);
}

TEST_F(ArduinoStubTest, LedcRecordsChannelState) {
  EXPECT_FALSE(arduino_stub::ledcChannel(1).setup);

  ledcSetup(1, 1000, 16);
  ledcAttachPin(15, 1);
  ledcWrite(1, 42);
  const arduino_stub::LedcChannel c = arduino_stub::ledcChannel(1);
  EXPECT_TRUE(c.setup);
  EXPECT_EQ(c.freq_hz, 1000u);
  EXPECT_EQ(c.resolution_bits, 16);
  EXPECT_EQ(c.pin, 15);
  EXPECT_TRUE(c.written);
  EXPECT_EQ(c.duty, 42u);
}

TEST_F(ArduinoStubTest, InterruptsCanBeFired) {
  EXPECT_FALSE(arduino_stub::fireInterrupt(18));

  int calls = 0;
  attachInterruptArg(18, countIsrArg, &calls, RISING);
  EXPECT_EQ(arduino_stub::interruptMode(18), RISING);
  EXPECT_TRUE(arduino_stub::fireInterrupt(18));
  EXPECT_TRUE(arduino_stub::fireInterrupt(18));
  EXPECT_EQ(calls, 2);

  g_isrCalls = 0;
  attachInterrupt(19, countIsr, CHANGE);
  EXPECT_TRUE(arduino_stub::fireInterrupt(19));
  EXPECT_EQ(g_isrCalls, 1);

  detachInterrupt(19);
  EXPECT_FALSE(arduino_stub::hasInterrupt(19));
}

TEST_F(ArduinoStubTest, ResetClearsAllState) {
  arduino_stub::setMicros(123);
  Serial.begin(9600);
  Serial.pushInput("x");
  Serial.write("y");
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  arduino_stub::setDigitalRead(3, HIGH);
  ledcSetup(1, 1000, 8);
  attachInterrupt(18, countIsr, RISING);

  arduino_stub::reset();

  EXPECT_EQ(micros(), 0UL);
  EXPECT_EQ(Serial.baud(), 0UL);
  EXPECT_EQ(Serial.available(), 0);
  EXPECT_EQ(Serial.output(), "");
  EXPECT_EQ(arduino_stub::pinModeOf(2), -1);
  EXPECT_EQ(arduino_stub::digitalWritten(2), -1);
  EXPECT_EQ(digitalRead(3), LOW);
  EXPECT_FALSE(arduino_stub::ledcChannel(1).setup);
  EXPECT_FALSE(arduino_stub::hasInterrupt(18));
}

TEST_F(ArduinoStubTest, CriticalSectionsCompileAsNoOps) {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  portEXIT_CRITICAL(&mux);
  SUCCEED();
}

} // namespace
