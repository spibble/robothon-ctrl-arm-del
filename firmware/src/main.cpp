#include <Arduino.h>
#include <ESP32Servo.h>

const int SERVO1_PIN   = 25;
const int SERVO2_PIN   = 32;
const int MIN_PULSE_US = 500;
const int MAX_PULSE_US = 2400;
const int REFRESH_MS   = 20; 

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  servo1.setPeriodHertz(50);  
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, MIN_PULSE_US, MAX_PULSE_US);
  servo2.attach(SERVO2_PIN, MIN_PULSE_US, MAX_PULSE_US);
}

void loop() {
  // test
  for (int angle = 0; angle <= 90; angle += 2) {
    servo1.write(angle);
    delay(REFRESH_MS);
  }
  delay(500);
  for (int angle = 0; angle <= 180; angle += 2) {
    servo2.write(angle);
    delay(REFRESH_MS);
  }
  delay(500);
  for (int angle = 90; angle >= 0; angle -= 2) {
    servo1.write(angle);
    delay(REFRESH_MS);
  }
  delay(500);
  for (int angle = 180; angle >= 0; angle -= 2) {
    servo2.write(angle);
    delay(REFRESH_MS);
  }
  delay(500);
}