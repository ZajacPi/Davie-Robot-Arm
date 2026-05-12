#include <Wire.h>

// =============================
// PCA9685 DEFINITIONS
// =============================
#define PCA9685_ADDR 0x40
#define MODE1        0x00
#define MODE2        0x01
#define LED0_ON_L    0x06
#define PRE_SCALE    0xFE

// =============================
// ESP32 I2C PINS
// =============================
#define SDA_PIN 26
#define SCL_PIN 25

// =============================
// SERVO CHANNELS
// =============================
#define JOINT5_CH 10
#define JOINT6_CH 11

// =============================
// PWM LIMITS (start conservative)
// =============================
int SERVO_MIN_US = 600;
int SERVO_CENTER_US = 1500;
int SERVO_MAX_US = 2400;

// =============================
// PCA9685 LOW LEVEL
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
  Wire.requestFrom(PCA9685_ADDR, 1);
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
  pca_write(MODE1, oldmode | 0xA1); // restart
}

void pca_set_pwm_us(uint8_t channel, int pulse_us) {
  uint16_t ticks = (uint16_t)(pulse_us * 50.0f * 4096.0f / 1000000.0f);

  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(LED0_ON_L + 4 * channel);
  Wire.write(0);           // ON_L
  Wire.write(0);           // ON_H
  Wire.write(ticks & 0xFF);
  Wire.write(ticks >> 8);
  Wire.endTransmission();
}

// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pca_write(MODE1, 0x00);
  pca_write(MODE2, 0x04);
  pca_set_pwm_freq(50); // 50 Hz for servos

  Serial.println("PCA9685 Servo Test Starting...");
}

// =============================
// LOOP
// =============================
void loop() {

  Serial.println("JOINT 5 → MIN");
  pca_set_pwm_us(JOINT5_CH, SERVO_MIN_US);
  delay(1500);

  Serial.println("JOINT 5 → CENTER");
  pca_set_pwm_us(JOINT5_CH, SERVO_CENTER_US);
  delay(1500);

  Serial.println("JOINT 5 → MAX");
  pca_set_pwm_us(JOINT5_CH, SERVO_MAX_US);
  delay(1500);

  Serial.println("JOINT 6 → MIN");
  pca_set_pwm_us(JOINT6_CH, SERVO_MIN_US);
  delay(1500);

  Serial.println("JOINT 6 → CENTER");
  pca_set_pwm_us(JOINT6_CH, SERVO_CENTER_US);
  delay(1500);

  Serial.println("JOINT 6 → MAX");
  pca_set_pwm_us(JOINT6_CH, SERVO_MAX_US);
  delay(3000);
}
