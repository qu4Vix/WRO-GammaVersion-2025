/*
 * Servo.h - Library for controling a servo with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */


/*
Our servo specifications:
MAX: (130) (150) (175) (143)
MIN: (50) (30) (55) (33)
1000us -> 2000us
*/

#ifndef CServo_h
#define CServo_h

#include <Arduino.h>
#include <ESP32Servo.h>

class CServo{  //maneja el servo
public:
  CServo(byte PinServo);
  void Attach();
  void MoveServo(int _angulo);
  int GetAngle();
  void BeginPWM();

private:
  Servo Miservo;
  ESP32PWM pwm;
  byte _pinServo;
  int _ang = 0;
  byte _servoMIN = 36;
  byte _servoMAX = 140;
};

#endif