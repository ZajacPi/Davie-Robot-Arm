#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/trigger.h>


// === Hall sensor ===
#define HALL_PIN 32 

// =============================
// STEPPER DRIVER PINS
// =============================
#define STEP_PIN 18
#define DIR_PIN 4
#define EN_PIN 23
#define M0_PIN 19
#define M1_PIN 21
#define M2_PIN 22

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
rcl_subscription_t sub_joints;
std_msgs__msg__Float32MultiArray joints_msg;

rcl_service_t home_srv;


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// =============================
// STEPPER SETUP
// =============================
const float stepsPerRevolution = 200;
int microstepSetting = 1;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

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
uint16_t angle_to_pwm(float angle, int min_us, int max_us) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  float pulse_us = min_us + (angle / 180.0) * (max_us - min_us);
  uint16_t ticks = (uint16_t)(pulse_us * 50 * 4096 / 1000000);
  return ticks;
}


// =============================
// JOINTS CALLBACK
// =============================
void joints_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *) msgin;

  if (msg->data.size < 6) return; // Require 6 joints

  for (int i = 1; i < 6; i++) {
    float angle_deg = msg->data.data[i];
    uint16_t ticks = angle_to_ticks(angle_deg, servo_min_us[i], servo_max_us[i]);
    pca_set_pwm(i, 0, ticks);
  }
}

// =============================
// STEPPER CALLBACK
// =============================
void home_callback(const void* req, void* res) {
  Serial.println("Homing stepper...");

  // Rotate slowly until hall sensor triggers
  stepper.setMaxSpeed(50);      // low speed for homing
  stepper.setAcceleration(100);
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);

  while (digitalRead(HALL_PIN) == HIGH) { // assuming HIGH = not triggered
    stepper.move(1);      // move one step at a time
    stepper.run();
  }

  stepper.setCurrentPosition(0); // set zero position
  Serial.println("Homing complete.");
}



// ==========================================================================================
// SETUP
// ==========================================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // --- Hall sensor ---
  pinMode(HALL_PIN, INPUT_PULLUP);

  // ---- I2C ----
  Wire.begin(SDA_PIN, SCL_PIN); // 400 kHz
  pca_write(MODE1, 0x00);
  pca_write(MODE2, 0x04); 
  pca_set_pwm_freq(50); // for all of the servos it is 50Hz

  Serial.println("PCA9685 initialized with register driver.");

  // --- Stepper setup ---
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  float MaxRPM = 300;
  float Max_Speed_StepsPerSec = microstepSetting * stepsPerRevolution * MaxRPM / 60;
  stepper.setMaxSpeed(Max_Speed_StepsPerSec);
  float AccelRPMperSec = 3000;
  float Accel_StepsPerSec2 = microstepSetting * stepsPerRevolution * AccelRPMperSec / 60;
  stepper.setAcceleration(Accel_StepsPerSec2);

  stepper.setCurrentPosition(0);

  Serial.println("Stepper driver configured.");

  ///////////////////////////////////////////// Mabye home on setup???

  // ---- micro-ROS Transport ----
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "controll_node", "", &support);

  // ---- Subscriber ----
  rclc_subscription_init_default(
      &sub_joints,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "joints_cmd"
  );
  
  // ---- Service ----
   rclc_service_init_default(
      &home_srv,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
      "home_stepper"
  );


  // ---- Executor ----
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub_joints, &joints_msg, &joints_callback, ON_NEW_DATA);
  rclc_executor_add_service(&executor, &home_srv, &handle_home_request);
  
  Serial.println("micro-ROS node ready.");
}

// =============================
// LOOP
// =============================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}