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
 * AdvancedMPU.h - Library for controling a MPU9250 and getting the overall angle with Arduino Framework.
 * 
 * Created by the Gamma Version Team, 2025
 * 
****************************************************/

#ifndef AdvancedMPU_h
#define AdvancedMPU_h

#include <Arduino.h>

#include <Wire.h>
#include <MPU9250.h>

class MPU {
    public:
    MPU();  // Class constructor
    void BeginWire(byte pinSDA, byte pinSCL, uint32_t frequency);   // Begin Wire 1
    void Setup();   // Set up the mpu 9265 with default settings in wire1
    void WorkOffset();  // Work out the offset of the mpu
    void MeasureFirstMicros();
    void UpdateAngle(); // Update the angle (Call in the loop)
    double GetAngle();   // Returns the angle
    void AddAngle(double _addedAngle);
    void SetDebugLedPin(uint8_t _pin);

    private:
    MPU9250 _mpu;   // MPU object
    byte _pinSDA;
    byte _pinSCL;
    byte _pinLED;
    double _offset;
    double _angle;
    uint32_t _prev_us_angle;
    bool _firstMicrosRead;
};

#endif