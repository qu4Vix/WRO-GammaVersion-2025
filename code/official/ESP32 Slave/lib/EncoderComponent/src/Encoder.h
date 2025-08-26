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
 * Encoder.h - Library for using an encoder with Arduino Framework.
 * 
****************************************************/



#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

class Encoder{
    public:
    Encoder(byte pinEncoder);
    void Attach(byte mode);
    long GetEncoder();
    long GetEncoderInterval();

    private:
    static Encoder* _sEncoder;
    static void EncoderISR();
    void UpdateEncoder();
    byte _pinEncoder;
    volatile long _encoder;
    volatile long _encoderTotal;
};

class Speedometer{
    public:
    Speedometer();
    
    private:
    int _speed;
};

#endif