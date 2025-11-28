#include <Arduino.h>
#include <ESP32Servo.h>

#define pinServo 25

#define servo0 85
#define servoRange 85
#define servoMAX servo0 + servoRange
#define servoMIN servo0 - servoRange

Servo miservo;

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  //miservo.setPeriodHertz(50);
  miservo.attach(pinServo, 600, 2400);
}

void loop() {
  if (Serial.available())
  {
    uint8_t data = Serial.parseInt();
    switch (data)
    {
    case 1:
      miservo.write(0);
      break;
    case 2:
      miservo.write(90);
      break;
    case 3:
      miservo.write(180);
      break;
    }
    
  }
  
}