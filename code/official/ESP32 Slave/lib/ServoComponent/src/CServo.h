/***************************************************
 * 
 * THE MAIN REPOSITORY CAN BE FOUND AT https://github.com/qu4Vix/WRO-GammaVersion-2025
 * 
 * This code is under a GPL-3.0 license. More information can be found in the License file
 * in the repository.
 * 
****************************************************/

/***************************************************
 * 
 * Servo.cpp - Library for controling a servo with Arduino Framework.
 * 
****************************************************/



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