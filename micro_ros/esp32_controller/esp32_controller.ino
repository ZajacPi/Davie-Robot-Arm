//RUN MICROROS
// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

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
// STATIC BUFFERS FOR JOINT MESSAGES 
static char name_data[6][20]; // 6 names, up to 19 chars + null
static rosidl_runtime_c__String name_seq[6]; // sequence elements
static double position_data[6];   

rcl_subscription_t sub_jointstate;
sensor_msgs__msg__JointState jointstate_msg;

rcl_service_t home_srv;
std_srvs__srv__Trigger_Request  home_request;
std_srvs__srv__Trigger_Response home_response;


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
volatile long target_stepper_position = 0;

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

   // update servo joints
  for (int j = 1; j <= 5; j++) {
      float angle_deg = msg->position.data[j] * 180.0f / M_PI;
      if(j == 2 || j == 3){
        angle_deg = -angle_deg;
      }
    
      int channel = 6 + j;  // j=1→7, j=5→11
      int idx = j - 1;      // servo_min/max indexed 0..4

      uint16_t ticks = angle_to_pwm(angle_deg,
                                    servo_min_us[idx],
                                    servo_max_us[idx]);
      pca_set_pwm(channel, 0, ticks);
  }


    //update stepper position
    float pos_rad = msg->position.data[0];

    // Convert radians to steps:
    // adjust gear ratio, microstepping, etc.
    float steps_per_rev = stepsPerRevolution * microstepSetting;
    long new_target = (long)(pos_rad * (steps_per_rev / (2.0f * M_PI)));

    target_stepper_position = new_target;

}


// =============================
// STEPPER CALLBACK (fixed to set response)
// =============================
void home_callback(const void* req, void* res) {
  std_srvs__srv__Trigger_Response * response = (std_srvs__srv__Trigger_Response *)res;
  home_stepper();
}

void home_stepper(){
  stepper.setMaxSpeed(50);
  stepper.setAcceleration(50);
  // stepper.enableOutputs();

  // unsigned long start = millis();
  while (digitalRead(HALL_PIN) == HIGH) { // assuming HIGH = not triggered
    stepper.move(5);      // move one step at a time
    stepper.run();

    // safety timeout in case sensor fails
    // if (millis() - start > 30000) {
    //   Serial.println("Homing timeout!");
    //   response->success = false;
    //   // rosidl_runtime_c__String__assign(&response->message, "Homing timeout");
    //   return;
    // }
  }
  stepper.setCurrentPosition(0);
  // stepper.disableOutputs();

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

  pca_set_pwm(11, 0, angle_to_pwm(0, FEETECH_MIN_US, FEETECH_MAX_US));
  delay(1000);

  pca_set_pwm(10, 0, angle_to_pwm(0, FEETECH_MIN_US, FEETECH_MAX_US));
  delay(1000);

  pca_set_pwm(9, 0, angle_to_pwm(0, FEETECH_MIN_US, FEETECH_MAX_US));
  delay(1000);

  pca_set_pwm(8, 0, angle_to_pwm(0, TOWERPRO_MIN_US, TOWERPRO_MAX_US));
  delay(1000);

  pca_set_pwm(7, 0, angle_to_pwm(0, TOWERPRO_MIN_US, TOWERPRO_MAX_US));
  delay(1000);


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
  // stepper.disableOutputs();  // quiet when idle


  float MaxRPM = 300;
  float Max_Speed_StepsPerSec = microstepSetting * stepsPerRevolution * MaxRPM / 60;
  stepper.setMaxSpeed(Max_Speed_StepsPerSec);
  float AccelRPMperSec = 3000;
  float Accel_StepsPerSec2 = microstepSetting * stepsPerRevolution * AccelRPMperSec / 60;
  stepper.setAcceleration(Accel_StepsPerSec2);

  stepper.setCurrentPosition(0);
  home_stepper();
  ///////////////////////////////////////////// Mabye home on setup???

  // ---- micro-ROS Transport ----
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp_controll_node", "", &support);

  sensor_msgs__msg__JointState__init(&jointstate_msg);

  // ---- Subscriber ----
  rclc_subscription_init_default(
      &sub_jointstate,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/esp_joint_states"
  );
  
  // ---- Service ----
   rclc_service_init_default(
      &home_srv,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
      "/esp_home_stepper"
  );

  // Allocate memory for JointState arrays (VERY IMPORTANT)
  jointstate_msg.name.data = name_seq;     // rosidl_runtime_c__String*
  jointstate_msg.name.size = 6;
  jointstate_msg.name.capacity = 6;
  // link each rosidl_runtime_c__String to a char buffer
  for (int i = 0; i < 6; i++) {
    name_seq[i].data = name_data[i];
    name_seq[i].size = 0;       // initially empty string
    name_seq[i].capacity = 20;  // room for 19 chars + '\0'
    name_data[i][0] = '\0';
  }

  // position sequence -> point to position_data
  jointstate_msg.position.data = position_data;
  jointstate_msg.position.size = 6;
  jointstate_msg.position.capacity = 6;
  // zero them initially
  for (int i = 0; i < 6; i++) position_data[i] = 0.0;

  // velocity and effort left empty
  jointstate_msg.velocity.data = NULL;
  jointstate_msg.velocity.size = 0;
  jointstate_msg.velocity.capacity = 0;

  jointstate_msg.effort.data = NULL;
  jointstate_msg.effort.size = 0;
  jointstate_msg.effort.capacity = 0;

  // ---- Executor ---- has 2 because topic and service
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_jointstate, &jointstate_msg, &jointstate_callback, ON_NEW_DATA);
  rclc_executor_add_service(&executor, &home_srv, &home_request, &home_response, &home_callback);
  }


void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  stepper.moveTo(target_stepper_position);  
  stepper.run();
  delay(1);
}