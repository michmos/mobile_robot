#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16.h>

rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_node_t node;

rcl_publisher_t publisher_ultrasonic;
rcl_publisher_t publisher_encoder;
rcl_subscription_t subscriber;
std_msgs__msg__Int16 subMessage;

rcl_timer_t timer_ultrasonic;
rcl_timer_t timer_rpm;

int8_t direction = 1;  // 1 forward, -1 backward
unsigned long pulseCount = 0;
const uint8_t encoderResolution = 12;
const uint8_t reducerRatio = 40;

// Motor driver
#define ENA_PIN 5
#define IN1_PIN 16
#define IN2_PIN 17

// Motor encoder
#define ENCODERA_1 18
#define ENCODERB_1 19


// ultrasonic sensor
#define ULTRASONIC_TRIGGER_PIN 2
#define ULTRASONIC_ECHO_PIN 4

#define RCCHECK(fn) \
  do { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  } while(0)

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
// Timer Callbacks
//////////////////////////////////////////////////////////////////////////////////
void on_encoder_a1_raise() {
  if (digitalRead(ENCODERB_1) == LOW) {
    direction = 1;
  } else {
    direction = -1;
  }
  pulseCount++;
}

// TODO: maybe use actual time difference (last_call_time)
void publish_rpm(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  unsigned long snaps = pulseCount;
  int8_t dir = direction;
  static unsigned long lastPulseCount = 0;

  // this substraction even works when pulseCount overflows (since the result
  // is casted to unsigned again)
  unsigned long pulseDiff = snaps - lastPulseCount;
  lastPulseCount = snaps;

  uint16_t RPM = (pulseDiff * 600) / (reducerRatio * encoderResolution);
  std_msgs__msg__Int16 msg;
  msg.data = RPM * dir;
  RCSOFTCHECK(rcl_publish(&publisher_encoder, &msg, NULL));
}

void publish_ultrasonic(rcl_timer_t *timer, int64_t last_call_time) {
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
  std_msgs__msg__UInt16 msg;
  msg.data = duration;
  RCSOFTCHECK(rcl_publish(&publisher_ultrasonic, &msg, NULL));
}

//////////////////////////////////////////////////////////////////////////////////
// Subscriptions
//////////////////////////////////////////////////////////////////////////////////
void subscriber_callback(const void *subMsg) {
  const std_msgs__msg__Int16 *message = (const std_msgs__msg__Int16 *)subMsg;

  int8_t direction = (message->data < 0) ? -1 : 1;
  uint16_t dutyCycle = map(abs(message->data), 0, INT16_MAX, 0, UINT16_MAX);

  // handle direction
  if (direction < 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }

  ledcWrite(1, dutyCycle);
}

//////////////////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////////////////
void setup() {
  set_microros_transports();

  // motor setup
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  // turn motor off
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  // pwm setup
  ledcSetup(1, 1000, 16);
  ledcAttachPin(ENA_PIN, 1);

  // encoder setup
  pinMode(ENCODERA_1, INPUT_PULLUP);
  pinMode(ENCODERB_1, INPUT_PULLUP);
  attachInterrupt(ENCODERA_1, on_encoder_a1_raise, RISING);

  // ultrasonic setup
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_ultrasonic,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
    "ultrasonic_raw"));

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

  // create timer to publish ultrasonic data
  RCCHECK(rclc_timer_init_default(
    &timer_ultrasonic,
    &support,
    RCL_MS_TO_NS(100),
    publish_ultrasonic));

  // TODO: maybe reuse same timer instead

  // create timer to publish encoder data
  RCCHECK(rclc_timer_init_default(
    &timer_rpm,
    &support,
    RCL_MS_TO_NS(100),
    publish_rpm));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_ultrasonic));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_rpm));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &subMessage, &subscriber_callback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin(&executor);
}
