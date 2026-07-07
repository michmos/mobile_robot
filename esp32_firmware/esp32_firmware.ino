#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 subMessage;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define ENA_PIN 23
#define IN1_PIN 19
#define IN2_PIN 21
#define ULTRASONIC_TRIGGER_PIN 16
#define ULTRASONIC_ECHO_PIN 17

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
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

// TODO: maybe this shouldn't happen inside the timer
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  // trigger sensor
  digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);

  // measure the duration until echo
  uint16_t duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 38000);
  if (duration == 0) {
    // 0 in case of timeout
    return;
  }
  uint16_t distance = (duration * 0.034) / 2;
  msg.data = distance;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

void subscriber_callback(const void *subMsg) {
  const std_msgs__msg__Int32 *message = (const std_msgs__msg__Int32 *)subMsg;
  // receives message between 0 - 65535
  uint16_t dutyCycle = message->data;
  if (message) {
    ledcWrite(1, dutyCycle);
  }
}

void setup() {
  set_microros_transports();

  // motor setup
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  // // turn motor off
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  // // pwm setup
  ledcSetup(1, 1000, 16);
  ledcAttachPin(ENA_PIN, 1);

  // ultrasonic setup
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ultra_sonic_data"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "pwm_control"));


  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &subMessage, &subscriber_callback, ON_NEW_DATA));

  // TODO: maybe don't set motor to max speed here but just wait for input from motor control
  // set motor to max speed
  digitalWrite(ENA_PIN, HIGH);
  // turn motor on
  digitalWrite(IN1_PIN, HIGH);
}

void loop() {
  rclc_executor_spin(&executor);
}
