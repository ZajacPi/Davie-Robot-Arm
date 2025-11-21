#include <Wire.h>



// === I2C pins for PCA9685 ===
#define SDA_PIN 26
#define SCL_PIN 25

// === PCA9685 address & registers ===
#define PCA9685_ADDR 0x40
#define MODE1        0x00
#define MODE2        0x01
#define LED0_ON_L    0x06
#define PRE_SCALE    0xFE

unsigned long start;


// === I2C helper functions ===
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
  uint8_t sleepmode = (oldmode & 0x7F) | 0x10;
  pca_write(MODE1, sleepmode);
  pca_write(PRE_SCALE, prescale);
  pca_write(MODE1, oldmode);
  delay(5);
  pca_write(MODE1, oldmode | 0xA1); // restart + auto-increment
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

// --- Map servo angle to PWM counts ---
uint16_t angle_to_pwm(float angle_deg) {
  float pulse_ms = 1.0 + (angle_deg / 180.0); // 1–2 ms
  uint16_t counts = (uint16_t)(pulse_ms / 20.0 * 4096); // for 50 Hz (20 ms period)
  return counts;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // --- I2C setup ---
  Wire.begin(SDA_PIN, SCL_PIN);
  pca_write(MODE1, 0x00);
  pca_write(MODE2, 0x04);
  pca_set_pwm_freq(50); // 50 Hz for servo
  Serial.println("PCA9685 initialized (register mode).");

}

void loop() {
  Serial.println("\n=== Test: Servo on PCA9685 (5 s) ===");
  start = millis();
  while (millis() - start < 5000) {
    for (int angle = 0; angle <= 180; angle += 5) {
      uint16_t off = angle_to_pwm(angle);
      pca_set_pwm(0, 0, off);
      delay(50);
    }
    for (int angle = 180; angle >= 0; angle -= 5) {
      uint16_t off = angle_to_pwm(angle);
      pca_set_pwm(0, 0, off);
      delay(50);
    }
  }
  Serial.println("Servo test done.\n");

  delay(3000); // pause before next cycle
}
