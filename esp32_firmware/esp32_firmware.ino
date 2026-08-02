#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16.h>

#include "inc/L298.hpp"
#include "inc/Encoder.hpp"


#define RCCHECK(fn) \
  do { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  } while (0)

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    delay(100);
  }
}

//////////////////////////////////////////////////////////////////////////////////
// Pins
//////////////////////////////////////////////////////////////////////////////////
// TODO: double check pins
// Motor driver
#define ENA_PIN 15
#define IN1_PIN 2
#define IN2_PIN 4

#define ENB_PIN 16
#define IN3_PIN 17
#define IN4_PIN 5

// Motor encoder
#define ENCODER1_SIGNAL1 18
#define ENCODER1_SIGNAL2 19

#define ENCODER2_SIGNAL1 21
#define ENCODER2_SIGNAL2 3

//////////////////////////////////////////////////////////////////////////////////
// Globals
//////////////////////////////////////////////////////////////////////////////////

// micro_ros
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rcl_publisher_t publisher_encoder;
rcl_subscription_t subscriber;
std_msgs__msg__Int16 sub_msg;
rcl_timer_t timer_rpm;

const uint8_t k_reducer_ratio = 40;
const uint8_t k_encoder_resolution = 12;
const uint8_t k_pwm_res = 16;

L298N g_motorDriver(ENA_PIN, IN1_PIN, IN2_PIN, ENB_PIN, IN3_PIN, IN4_PIN);
Encoder g_encoder1(ENCODER1_SIGNAL1, ENCODER1_SIGNAL2);
Encoder g_encoder2(ENCODER2_SIGNAL1, ENCODER2_SIGNAL2);

//////////////////////////////////////////////////////////////////////////////////
// Timer Callbacks
//////////////////////////////////////////////////////////////////////////////////

// TODO: adapt to both motors
void publish_rpm(rcl_timer_t *timer, int64_t last_call_time) {
  // RCLC_UNUSED(last_call_time);
  //
  // unsigned long pulse_diff = g_encoder1.get_pulse_diff();
  //
  // uint16_t RPM = (pulse_diff * 600) / (k_reducer_ratio * k_encoder_resolution);
  // std_msgs__msg__Int16 msg;
  // msg.data = RPM * dir;
  // RCSOFTCHECK(rcl_publish(&publisher_encoder, &msg, NULL));
}

//////////////////////////////////////////////////////////////////////////////////
// Subscriptions
//////////////////////////////////////////////////////////////////////////////////
// TODO: adapt to both motors
void subscriber_callback(const void *subMsg) {
  // const std_msgs__msg__Int16 *message = (const std_msgs__msg__Int16 *)subMsg;
  //
  // e_direction dir = (message->data < 0) ? BACKWARDS : FORWARDS;
  // uint16_t dutyCycle = map(abs(message->data), 0, INT16_MAX, 0, UINT16_MAX);
  //
  // g_motorDriver.updateDirection1(dir);
  // g_motorDriver.updateSpeed(dutyCycle, dutyCycle);
}

//////////////////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////////////////
void setup() {
  set_microros_transports();

  g_motorDriver.setup(k_pwm_res);
  g_encoder1.setup();
  g_encoder2.setup();

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_encoder,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "rpm_raw"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "pwm_control"));

  // create timer to publish encoder data
  RCCHECK(rclc_timer_init_default(
    &timer_rpm,
    &support,
    RCL_MS_TO_NS(100),
    publish_rpm));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_rpm));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscriber_callback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin(&executor);
}
