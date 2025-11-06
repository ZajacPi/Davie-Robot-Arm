#include <Wire.h>
#include <AccelStepper.h>

// === Stepper driver pins ===
#define STEP_PIN 18
#define DIR_PIN 4
#define EN_PIN 23
#define M0_PIN 19
#define M1_PIN 21
#define M2_PIN 22

// === Hall sensor ===
#define HALL_PIN 32  // input-only, perfect for Hall sensor

// === I2C pins for PCA9685 ===
#define SDA_PIN 26
#define SCL_PIN 25

// === PCA9685 address & registers ===
#define PCA9685_ADDR 0x40
#define MODE1        0x00
#define MODE2        0x01
#define LED0_ON_L    0x06
#define PRE_SCALE    0xFE

// === Stepper setup ===
const float stepsPerRevolution = 200;
int microstepSetting = 1;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

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

  // --- Hall sensor ---
  pinMode(HALL_PIN, INPUT_PULLUP);

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
  Serial.println("Setup complete.\n");
}

void loop() {
  // --- Test 1: Hall Sensor ---
  Serial.println("=== Test 1: Hall Sensor (5 s) ===");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    int hallState = digitalRead(HALL_PIN);
    Serial.println(hallState == LOW ? "Magnet detected" : "No magnet");
    delay(200);
  }

  // --- Test 2: Stepper motor ---
  Serial.println("\n=== Test 2: Stepper Motor (5 s) ===");
  start = millis();
  while (millis() - start < 5000) {
    stepper.moveTo(stepsPerRevolution);  // 1 rev
    while (stepper.distanceToGo() != 0 && (millis() - start < 2500))
      stepper.run();

    stepper.moveTo(0);  // back to zero
    while (stepper.distanceToGo() != 0 && (millis() - start < 5000))
      stepper.run();
  }
  stepper.disableOutputs();
  Serial.println("Stepper test done.");

  // --- Test 3: Servo movement ---
  Serial.println("\n=== Test 3: Servo on PCA9685 (5 s) ===");
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
