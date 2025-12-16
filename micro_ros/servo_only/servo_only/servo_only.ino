#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_srvs/srv/trigger.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================
// I2C PINS (ESP32)
// =============================
#define SDA_PIN 26
#define SCL_PIN 25

// =============================
// PCA9685 REGISTER DEFINITIONS
// =============================
#define PCA9685_ADDR 0x40
#define MODE1        0x00
#define MODE2        0x01
#define LED0_ON_L    0x06
#define PRE_SCALE    0xFE

// =============================
// SERVO JOINTS
// =============================
// feetech
#define JOINT2 0
#define JOINT3 1
#define JOINT4 2
// towerpro
#define JOINT5 3
#define JOINT6 4

const int FEETECH_MIN_US = 500;   
const int FEETECH_MAX_US = 2500;  

const int TOWERPRO_MIN_US = 1000;   
const int TOWERPRO_MAX_US = 2000;  

// TIMING TABLE by channel
int servo_min_us[5] = {
  FEETECH_MIN_US,  // joint2
  FEETECH_MIN_US,  // joint3
  FEETECH_MIN_US,  // joint4
  TOWERPRO_MIN_US, // joint5
  TOWERPRO_MIN_US  // joint6
};

int servo_max_us[5] = {
  FEETECH_MAX_US,
  FEETECH_MAX_US,
  FEETECH_MAX_US,
  TOWERPRO_MAX_US,
  TOWERPRO_MAX_US
};

// =============================
// micro-ROS VARIABLES
// =============================
rcl_subscription_t sub_jointstate;
sensor_msgs__msg__JointState jointstate_msg;
sensor_msgs__msg__JointState__init(&jointstate_msg);

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
  Wire.endTransmission();}

uint8_t pca_read(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
  return Wire.read();}

void pca_set_pwm_freq(float freq_hz) {
  float prescaleval = 25000000.0 / (4096.0 * freq_hz) - 1.0;
  uint8_t prescale = (uint8_t)(prescaleval + 0.5);
  uint8_t oldmode = pca_read(MODE1);
  pca_write(MODE1, (oldmode & 0x7F) | 0x10); // sleep
  pca_write(PRE_SCALE, prescale);
  pca_write(MODE1, oldmode);
  delay(5);
  pca_write(MODE1, oldmode | 0xA1); }// restart + AI bit

void pca_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(LED0_ON_L + 4 * channel);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();}

uint16_t angle_to_pwm(float angle, int min_us, int max_us) {
  angle += 90.0; 
  if (angle < 0.0f) angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  float pulse_us = min_us + (angle / 180.0) * (max_us - min_us);
  uint16_t ticks = (uint16_t)(pulse_us * 50 * 4096 / 1000000);
  return ticks;
}


// =============================
// JOINTS CALLBACK
// =============================
void jointstate_callback(const void * msgin)
{
    const sensor_msgs__msg__JointState * msg =(const sensor_msgs__msg__JointState *)msgin;

    if (msg->position.size < 6) return;
    static unsigned long last_update = 0;
    if (millis() - last_update < 20) return; // ~50 Hz update rate
    last_update = millis();

    //update servo joints
    for (int i = 0; i < 5; i++) {
        float angle_deg = msg->position.data[i+1] * 180.0 / M_PI;
        uint16_t ticks = angle_to_pwm(angle_deg, servo_min_us[i], servo_max_us[i]);
        pca_set_pwm(i, 0, ticks);
    }
}



// ==========================================================================================
// SETUP
// ==========================================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- I2C ----
  Wire.begin(SDA_PIN, SCL_PIN); // 400 kHz
  pca_write(MODE1, 0x00);
  pca_write(MODE2, 0x04); 
  pca_set_pwm_freq(50); // for all of the servos it is 50Hz
  
  // direct sweep test on channel 0
  pca_set_pwm(0, 0, angle_to_pwm(-90, FEETECH_MIN_US, FEETECH_MAX_US));
  delay(500);
  pca_set_pwm(0, 0, angle_to_pwm(0, FEETECH_MIN_US, FEETECH_MAX_US));
  delay(500);
  pca_set_pwm(0, 0, angle_to_pwm(180, FEETECH_MIN_US, FEETECH_MAX_US));
  delay(500);

  // ---- micro-ROS Transport ----
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp_controll_node", "", &support);

  // ---- Subscriber ----
  rclc_subscription_init_default(
      &sub_jointstate,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/esp_joint_states"
  );
  

  // Allocate memory for JointState arrays (VERY IMPORTANT)
// Allocate name[]
jointstate_msg.name.capacity = 6;
jointstate_msg.name.size = 6;
jointstate_msg.name.data = malloc(6 * sizeof(rosidl_runtime_c__String));

for(int i = 0; i < 6; i++) {
    jointstate_msg.name.data[i].capacity = 20;
    jointstate_msg.name.data[i].size = 0;
    jointstate_msg.name.data[i].data = malloc(20);
}

// Allocate position[]
jointstate_msg.position.capacity = 6;
jointstate_msg.position.size = 6;
jointstate_msg.position.data = malloc(6 * sizeof(double));

// velocity[] and effort[] are EMPTY ARRAYS (size=0), so sub should accept them.
jointstate_msg.velocity.capacity = 0;
jointstate_msg.velocity.size = 0;
jointstate_msg.velocity.data = NULL;

jointstate_msg.effort.capacity = 0;
jointstate_msg.effort.size = 0;
jointstate_msg.effort.data = NULL;


  // ---- Executor ----
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub_jointstate, &jointstate_msg, &jointstate_callback, ON_NEW_DATA);
  }

// =============================
// LOOP
// =============================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}