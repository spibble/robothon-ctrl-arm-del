/*
#include <Arduino.h>
#include <ESP32Servo.h>

// Servos 1-5 are 180-degree SG90 microservos. Servo 6 is a continuous rotation MG996R servo.
const int SERVO1_PIN   = 12;
const int SERVO2_PIN   = 14;
const int SERVO3_PIN   = 27;
const int SERVO4_PIN   = 26;
const int SERVO5_PIN   = 33;
const int SERVO6_PIN   = 32;


const int MIN_PULSE_US = 500;
const int MAX_PULSE_US = 2400;
const int REFRESH_MS   = 20; 

Servo servo1, servo2, servo3, servo4, servo5, servo6;

void setup() {
  Serial.begin(115200);
  servo1.setPeriodHertz(50);  
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);  
  servo4.setPeriodHertz(50);
  servo5.setPeriodHertz(50);
  servo6.setPeriodHertz(50);

  servo1.attach(SERVO1_PIN, MIN_PULSE_US, MAX_PULSE_US);
  servo2.attach(SERVO2_PIN, MIN_PULSE_US, MAX_PULSE_US);
  servo3.attach(SERVO3_PIN, MIN_PULSE_US, MAX_PULSE_US);
  servo4.attach(SERVO4_PIN, MIN_PULSE_US, MAX_PULSE_US);
  servo5.attach(SERVO5_PIN, MIN_PULSE_US, MAX_PULSE_US);
  servo6.attach(SERVO6_PIN, MIN_PULSE_US, MAX_PULSE_US);
}

void loop() {
  // test

  servo1.write(180);
  servo2.write(180);
  servo3.write(180);
  servo4.write(180);
  servo5.write(180);
  servo6.writeMicroseconds(1700);
  delay(1000);
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);
  servo5.write(0);
  servo6.writeMicroseconds(1300);
  delay(1000);
}
*/