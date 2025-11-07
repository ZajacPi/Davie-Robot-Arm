// wiring looking from the front of the sensor: 
// 3.3V
// GND
// Resistor 10k (or 4,7k if a lto of noise) to 3.3V, and to the reading pin
#define HALL_PIN 15  

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT); 
}

void loop() {
  int state = digitalRead(HALL_PIN);
  Serial.println(state == LOW ? "MAGNET DETECTED" : "NO MAGNET");
  delay(200);
}