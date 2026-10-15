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
// Motor driver
#define ENA_PIN 15
#define IN1_PIN 2
#define IN2_PIN 4

#define IN3_PIN 16
#define IN4_PIN 17
#define ENB_PIN 5

// Motor encoder
#define ENCODER1_SIGNAL1 18
#define ENCODER1_SIGNAL2 19

#define ENCODER2_SIGNAL1 22
#define ENCODER2_SIGNAL2 23

//////////////////////////////////////////////////////////////////////////////////
// Globals
//////////////////////////////////////////////////////////////////////////////////

// constants
const uint8_t k_pwm_res = 16;

// locks
portMUX_TYPE g_encoder_mux = portMUX_INITIALIZER_UNLOCKED;

SerialChannel g_sc;
L298N g_motorDriver(ENA_PIN, IN1_PIN, IN2_PIN, ENB_PIN, IN3_PIN, IN4_PIN, k_pwm_res);
Encoder g_encoder1(ENCODER1_SIGNAL1, ENCODER1_SIGNAL2);
Encoder g_encoder2(ENCODER2_SIGNAL1, ENCODER2_SIGNAL2);
MotorControl g_motorController;
// config update, command watchdog and FF + PI control, see inc/Drive.hpp
Drive g_drive(g_sc, g_motorDriver, g_encoder1, g_encoder2, g_motorController);

//////////////////////////////////////////////////////////////////////////////////
// Sensor data
//////////////////////////////////////////////////////////////////////////////////

// use portENTER_CRITICAL to disable interrupts ensuring all sample data to be taken
// around the same time
EncoderSample
sampleEncoders() {
  EncoderSample s;
  portENTER_CRITICAL(&g_encoder_mux);
  s.left_ticks = g_encoder1.get_tick_count();
  s.right_ticks = g_encoder2.get_tick_count();
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

void setup() {
  Serial.begin(115200);
  g_motorDriver.setup();
  g_encoder1.setup();
  g_encoder2.setup();
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
