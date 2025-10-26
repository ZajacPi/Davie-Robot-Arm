#include <AccelStepper.h>

// Stepper driver pins (safe GPIOs for ESP32)
#define STEP_PIN 18
#define DIR_PIN 4

// Microstepping control pins
#define M0_PIN 19
#define M1_PIN 21
#define M2_PIN 22

#define EN_PIN 23

const float stepsPerRevolution = 200;
int microstepSetting = 1;
// int microstepSetting = 16;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

float convert_rotational_position_to_steps(float rotations) {
  return rotations * stepsPerRevolution * microstepSetting;
}

void setup() {
  Serial.begin(115200);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  // digitalWrite(M2_PIN, HIGH); // use for microstepping if desired

  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);  // enable is active LOW
  stepper.enableOutputs();

  float MaxRPM = 300;
  float Max_Speed_StepsPerSec = microstepSetting * stepsPerRevolution * MaxRPM / 60;
  stepper.setMaxSpeed(Max_Speed_StepsPerSec);

  float AccelRPMperSec = 3000;
  float Accel_StepsPerSec2 = microstepSetting * stepsPerRevolution * AccelRPMperSec / 60;
  stepper.setAcceleration(Accel_StepsPerSec2);

  stepper.setCurrentPosition(0);
}

void loop() {
  float positions[] = {0.25, 0.75, 0.5, 1.0, 0.0};

  for (int i = 0; i < 5; i++) {
    stepper.enableOutputs();
    stepper.moveTo(convert_rotational_position_to_steps(positions[i]));
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    delay(200);
    stepper.disableOutputs();  // quiet when idle
    delay(1000);
  }

  delay(2000);
}
