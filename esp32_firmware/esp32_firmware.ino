#include <stdio.h>
#include "inc/L298.hpp"
#include "inc/Encoder.hpp"
#include "inc/SerialChannel.hpp"
#include "inc/Messages.hpp"
#include "inc/MotorControl.hpp"

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
// Types
//////////////////////////////////////////////////////////////////////////////////
struct EncoderSample {
  int32_t left_ticks = 0;
  int32_t right_ticks = 0;
  unsigned long us = 0;
};
//////////////////////////////////////////////////////////////////////////////////
// Globals
//////////////////////////////////////////////////////////////////////////////////

// constants
const uint8_t k_pwm_res = 16;

// locks
portMUX_TYPE g_encoder_mux = portMUX_INITIALIZER_UNLOCKED;

bool g_wasStopped = true;  //indicates if the motors were stopped in the last control
                           //loop iteration

msgs::Config g_config;
SerialChannel g_sc;
L298N g_motorDriver(ENA_PIN, IN1_PIN, IN2_PIN, ENB_PIN, IN3_PIN, IN4_PIN, k_pwm_res);
Encoder g_encoder1(ENCODER1_SIGNAL1, ENCODER1_SIGNAL2);
Encoder g_encoder2(ENCODER2_SIGNAL1, ENCODER2_SIGNAL2);
MotorControl g_motorController;

//////////////////////////////////////////////////////////////////////////////////
// Control Motor
//////////////////////////////////////////////////////////////////////////////////

float ticksToRad(int32_t ticks) {
  return (ticks * 2.0f * M_PI) / g_config.ticks_per_wheel_rev;
}

// returns the velocity in radians per second
float getVelocity(int32_t ticksNow, int32_t ticksLast, unsigned long dt_us) {
  return ticksToRad(ticksNow - ticksLast) / (dt_us * 1e-6f);
}

// update the motor driver with the latest command from the serial channel converted
// to a duty cycle using the FF + PI controller
void updateMotor(const EncoderSample& e) {
  // compute current velocities
  static EncoderSample lastSample;

  msgs::MotorCmd cmd = g_sc.getMotorCmd();
  if (millis() - cmd.timestamp_ms > g_config.watchdog_timeout_ms) {
    g_motorDriver.stop();
    g_motorController.reset();
    g_wasStopped = true;
    g_sc.writeln(msgs::Log("No new motor command received, stopping motors"));
    return;
  }

  float dutyLeft, dutyRight;
  if (g_wasStopped) {
    // ff without pi to avoid integral windup
    dutyLeft = g_motorController.feedforwardLeft(cmd.left_vel_cmd);
    dutyRight = g_motorController.feedforwardRight(cmd.right_vel_cmd);

  } else {
    // get current velocity
    float velLeft = getVelocity(e.left_ticks, lastSample.left_ticks, e.us - lastSample.us);
    float velRight = getVelocity(e.right_ticks, lastSample.right_ticks, e.us - lastSample.us);
    float dt_s = (e.us - lastSample.us) * 1e-6f;

    dutyLeft = g_motorController.computeDutyLeft(cmd.left_vel_cmd, velLeft, dt_s);
    dutyRight = g_motorController.computeDutyRight(cmd.right_vel_cmd, velRight, dt_s);
  }
  g_wasStopped = false;
  lastSample = e;

  // update direction
  g_motorDriver.updateDir1((dutyLeft >= 0.0f) ? FORWARDS : BACKWARDS);
  g_motorDriver.updateDir2((dutyRight >= 0.0f) ? FORWARDS : BACKWARDS);
  // update duty
  if (g_motorDriver.updateDuty(fabsf(dutyLeft), fabsf(dutyRight)) == -1) {
    g_sc.writeln(msgs::Log("Could not update motor duty cycles"));
  }
}

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

// adopt a newly received configuration into g_config
// send ACK on valid new config
// @return: true if a new configuration was applied
bool updateConfig() {
  msgs::Config received = g_sc.getConfig();

  // nothing to do unless the channel parsed a configuration we haven't seen -
  if (!received.initialized || received.timestamp_ms == g_config.timestamp_ms) {
    return false;
  }

  g_motorDriver.stop();
  g_wasStopped = true;

  // update component configs
  g_motorController.configure(received.min_duty_left, received.min_duty_right, received.slope_left, received.slope_right, received.kp, received.ki, received.integral_limit);
  g_motorDriver.setInverted1(received.invert_left);
  g_motorDriver.setInverted2(received.invert_right);
  g_encoder1.setInverted(received.invert_left);
  g_encoder2.setInverted(received.invert_right);

  g_config = received;

  g_sc.writeln(msgs::Log("New configuration received"));
  g_sc.writeln(msgs::Event(msgs::Event::ACK));
  return true;
}

// wait for the pi to answer the SETUP event with a configuration
// @param timeout_ms: how long to keep trying before giving up
// @return: true if a configuration was received in time
bool waitForConfig(unsigned long timeout_ms) {
  unsigned long start_ms = millis();

  while (millis() - start_ms < timeout_ms) {
    g_sc.read();
    if (updateConfig()) {
      return true;
    }
    delay(10);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  // send setup message for hardware plugin to resend configurations such as PI gains
  g_sc.writeln(msgs::Event(msgs::Event::SETUP));

  g_motorDriver.setup();
  g_encoder1.setup();
  g_encoder2.setup();

  // never block forever on the pi - falling back to the compiled-in defaults
  const unsigned long k_config_timeout_ms = 5000;
  if (!waitForConfig(k_config_timeout_ms)) {
    g_sc.writeln(msgs::Log("No configuration received within " + std::to_string(k_config_timeout_ms) + " ms, continuing with defaults"));
  }

  g_motorController.configure(g_config.min_duty_left, g_config.min_duty_right, g_config.slope_left, g_config.slope_right, g_config.kp, g_config.ki, g_config.integral_limit);
}

//////////////////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////////////////

// returns true when next has been reached and updates next to the next time to run
// otherwise just returns false
bool due(unsigned long now, unsigned long& next, unsigned long period_us) {
  if ((long)(now - next) < 0) {
    return false;
  }

  // update next by period_us and reset it in case more than one period has passed
  next = (now - next > period_us) ? now + period_us : next + period_us;
  return true;
}

void loop() {
  static unsigned long next_report_us = micros();
  static unsigned long next_control_us = next_report_us;
  static EncoderSample es;

  g_sc.read();

  if (updateConfig()) {  // reset timers since frequencies might have changed
    next_report_us = micros();
    next_control_us = next_report_us;
  }

  unsigned long now = micros();
  bool control_due = due(now, next_control_us, 1000000UL / g_config.control_rate_hz);
  bool report_due = due(now, next_report_us, 1000000UL / g_config.report_rate_hz);

  if (control_due || report_due) {
    es = sampleEncoders();
    if (control_due) {
      updateMotor(es);
    }
    if (report_due) {
      sendSensorData(es);
    }
  }
}
