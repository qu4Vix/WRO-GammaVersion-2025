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



#include "CServo.h"

CServo::CServo(byte PinServo) {
  _pinServo = PinServo;
}

void CServo::BeginPWM() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
}

void CServo::Attach(uint16_t servoMIN, uint16_t servoMAX) {
  Miservo.setPeriodHertz(50);
  Miservo.attach(_pinServo, servoMIN, servoMAX);
}

void CServo::Attach() {
  Attach(1000, 2000);
}

void CServo::MoveServo(int angle) {
  _ang = constrain(angle, 0, 180);
  Miservo.write(_ang);
}

void CServo::MoveSteeringServo(int angle) {
  _ang = map(angle, -90, 90, _servoMIN, _servoMAX);
  _ang = constrain(_ang, _servoMIN, _servoMAX);
  Miservo.write(_ang);
}

int CServo::GetAngle() {
  return _ang;
}