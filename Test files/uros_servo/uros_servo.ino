#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// =============================
// PCA9685 REGISTER DEFINITIONS
// =============================
#define PCA9685_ADDR 0x40
#define MODE1        0x00
#define MODE2        0x01
#define LED0_ON_L    0x06
#define PRE_SCALE    0xFE

// =============================
// I2C PINS (ESP32)
// =============================
#define SDA_PIN 26
#define SCL_PIN 25

// =============================
// SERVO SETTINGS
// =============================
#define SERVO_CHANNEL 0
const int SERVO_MIN_US = 500;   // modify for your servo
const int SERVO_MAX_US = 2500;  // modify for your servo

// =============================
// micro-ROS VARIABLES
// =============================
rcl_subscription_t servo_sub;
std_msgs__msg__Float32 servo_msg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// =============================
// LOW LEVEL PCA9685 FUNCTIONS
// =============================

void pca_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t pca_read(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(PCA9685_ADDR, (uint8_t)1);
  return Wire.read();
}

void pca_set_pwm_freq(float freq_hz) {
  float prescaleval = 25000000.0 / (4096.0 * freq_hz) - 1.0;
  uint8_t prescale = (uint8_t)(prescaleval + 0.5);

  uint8_t oldmode = pca_read(MODE1);
  pca_write(MODE1, (oldmode & 0x7F) | 0x10); // sleep
  pca_write(PRE_SCALE, prescale);
  pca_write(MODE1, oldmode);
  delay(5);
  pca_write(MODE1, oldmode | 0xA1); // restart + AI bit
}

void pca_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(LED0_ON_L + 4 * channel);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

// =============================
// ANGLE TO PWM TICKS
// =============================
uint16_t angle_to_pwm(float angle_deg) {
  if (angle_deg < 0) angle_deg = 0;
  if (angle_deg > 180) angle_deg = 180;

  float pulse_us = SERVO_MIN_US + (angle_deg / 180.0) * (SERVO_MAX_US - SERVO_MIN_US);
  uint16_t ticks = (uint16_t)(pulse_us * 50 * 4096 / 1000000);
  return ticks;
}

// =============================
// SERVO CALLBACK
// =============================
void servo_callback(const void * msgin) {
  const std_msgs__msg__Float32 * m = (const std_msgs__msg__Float32 *) msgin;

  uint16_t ticks = angle_to_pwm(m->data);
  pca_set_pwm(SERVO_CHANNEL, 0, ticks);
}

// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- I2C ----
  Wire.begin(SDA_PIN, SCL_PIN); // 400 kHz
  pca_write(MODE1, 0x00);
  pca_write(MODE2, 0x04); 
  pca_set_pwm_freq(50);

  Serial.println("PCA9685 initialized with register driver.");

  // ---- micro-ROS Transport ----
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pca9685_register_node", "", &support);

  // ---- Subscriber ----
  rclc_subscription_init_default(
      &servo_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "servo_angle"
  );

  // ---- Executor ----
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);

  Serial.println("micro-ROS PCA9685 node ready.");
}

// =============================
// LOOP
// =============================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}
