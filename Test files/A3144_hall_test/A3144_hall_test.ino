#define HALL_PIN 32  // GPIO where A3144 output is connected

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT);  // If using bare sensor, use INPUT_PULLUP
  Serial.println("A3144 Hall Sensor Test Started");
}

void loop() {
  int hallState = digitalRead(HALL_PIN);

  if (hallState == LOW) {
    Serial.println("Magnet detected!");
  } else {
    Serial.println("No magnet detected");
  }

  delay(500);
}