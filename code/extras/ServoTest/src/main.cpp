#include <Arduino.h>
#include <ESP32Servo.h>

#define pinServo 13

#define servo0 88
#define servoRange 52
#define servoMAX servo0 + servoRange
#define servoMIN servo0 - servoRange

Servo miservo;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  miservo.setPeriodHertz(50);
  miservo.attach(pinServo, 1000, 2000);
}

void loop() {
  miservo.write(servoMAX);
  delay(10000);
  miservo.write(servoMIN);
  delay(10000);
  miservo.write(servo0);
  delay(10000);
}