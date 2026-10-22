#include <stdio.h>
#include "inc/L298.hpp"
#include "inc/Encoder.hpp"
#include "inc/SerialChannel.hpp"
#include "inc/Messages.hpp"
#include "inc/MotorControl.hpp"
#include "inc/Drive.hpp"
#include "inc/Scheduler.hpp"

//////////////////////////////////////////////////////////////////////////////////
// Pins
//////////////////////////////////////////////////////////////////////////////////
// TODO: double check pins
// Motor driver, L298N board labels in the comments. The right wheel is on
// output A, the left wheel on output B
#define LEFT_EN_PIN 5    // ENB
#define LEFT_IN1_PIN 16  // IN3
#define LEFT_IN2_PIN 17  // IN4

#define RIGHT_EN_PIN 15  // ENA
#define RIGHT_IN1_PIN 2  // IN1
#define RIGHT_IN2_PIN 4  // IN2

// Motor encoder
#define LEFT_ENCODER_SIGNAL1 22
#define LEFT_ENCODER_SIGNAL2 23

#define RIGHT_ENCODER_SIGNAL1 18
#define RIGHT_ENCODER_SIGNAL2 19

//////////////////////////////////////////////////////////////////////////////////
// Globals
//////////////////////////////////////////////////////////////////////////////////

// constants
const uint8_t k_pwm_res = 16;

// locks
portMUX_TYPE g_encoder_mux = portMUX_INITIALIZER_UNLOCKED;

SerialChannel g_sc;
// motor 1 of the driver is the left wheel, motor 2 the right one (Drive.hpp)
L298N g_motorDriver(LEFT_EN_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, RIGHT_EN_PIN,
                    RIGHT_IN1_PIN, RIGHT_IN2_PIN, k_pwm_res);
Encoder g_encoderLeft(LEFT_ENCODER_SIGNAL1, LEFT_ENCODER_SIGNAL2);
Encoder g_encoderRight(RIGHT_ENCODER_SIGNAL1, RIGHT_ENCODER_SIGNAL2);
MotorControl g_motorController;
// config update, command watchdog and FF + PI control, see inc/Drive.hpp
Drive g_drive(g_sc, g_motorDriver, g_encoderLeft, g_encoderRight, g_motorController);

//////////////////////////////////////////////////////////////////////////////////
// Sensor data
//////////////////////////////////////////////////////////////////////////////////

// use portENTER_CRITICAL to disable interrupts ensuring all sample data to be taken
// around the same time
EncoderSample
sampleEncoders() {
  EncoderSample s;
  portENTER_CRITICAL(&g_encoder_mux);
  s.left_ticks = g_encoderLeft.get_tick_count();
  s.right_ticks = g_encoderRight.get_tick_count();
  s.us = micros();
  portEXIT_CRITICAL(&g_encoder_mux);
  return s;
}

void sendSensorData(const EncoderSample& e) {
  msgs::EncoderData ed(e.left_ticks, e.right_ticks, e.us);
  g_sc.writeln(ed);
}

// ImuData getImuData() {
// }

//////////////////////////////////////////////////////////////////////////////////
// Configure
//////////////////////////////////////////////////////////////////////////////////

// sends SETUP every k_setup_interval_ms until a valid configuration is
// received and applied, the serial port is read in between
void awaitConfig() {
  // the pi re-sends the config on its own, SETUP only has to reach it once
  const unsigned long k_setup_interval_ms = 500;
  unsigned long last_setup_ms = millis() - k_setup_interval_ms; // send now
  do {
    if (millis() - last_setup_ms >= k_setup_interval_ms) {
      g_sc.writeln(msgs::Event(msgs::Event::SETUP));
      last_setup_ms = millis();
    }
    g_sc.read();
    delay(10);
  } while (!g_drive.updateConfig());
}

//////////////////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////////////////

// without pwm the motors can't move, so the firmware stops here instead of
// asking for a configuration. The direction pins stay LOW, neither motor is
// driven. The log line is repeated because the pi may open the port later.
// The pi logs it, and its handshake times out because no config is ACKed
void haltOnMotorDriverFailure() {
  while (true) {
    g_sc.writeln(msgs::Log("Motor pwm setup (ledcAttach) failed, halted"));
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  if (!g_motorDriver.setup()) {
    haltOnMotorDriverFailure();
  }
  g_encoderLeft.setup();
  g_encoderRight.setup();
  g_motorDriver.brake();

  awaitConfig();
}

void loop() {
  static uint32_t next_report_us = micros();
  static uint32_t next_control_us = next_report_us;
  static EncoderSample es;

  g_sc.read();

  if (g_drive.updateConfig()) {  // reset timers since frequencies might have changed
    next_report_us = micros();
    next_control_us = next_report_us;
  }

  uint32_t now = micros();
  bool control_due = due(now, next_control_us, 1000000UL / g_drive.config().control_rate_hz);
  bool report_due = due(now, next_report_us, 1000000UL / g_drive.config().report_rate_hz);

  if (control_due || report_due) {
    es = sampleEncoders();
    if (control_due) {
      g_drive.updateMotor(es);
    }
    if (report_due) {
      sendSensorData(es);
    }
  }
}
