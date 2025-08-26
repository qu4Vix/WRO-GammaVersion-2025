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
 * Motor.cpp - Library for controling an engine with Arduino Framework.
 * 
****************************************************/



#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor{  
  public:
  Motor(byte PinPWM, byte PinDir1, byte PinDir2, byte PinEnable, float kp, float kd);
  void Init();
  void SetPower(int pot);
  void SetSpeed(int actualSpeed, int targetSpeed);
  float GetPower();

  private:
  byte _pinPWM;
  byte _pinDir1;
  byte _pinDir2;
  byte _pinEnable;
  float _power = 0;
  float _error;
  float _lastError;
  float _kp;
  float _kd;
  bool _forward;
};

#endif