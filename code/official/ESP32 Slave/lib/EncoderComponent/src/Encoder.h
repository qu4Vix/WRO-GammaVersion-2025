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
    unsigned long GetEncoder();
    long GetEncoderInterval();
    void SetMotionDirection(bool forward);

    private:
    static Encoder* _sEncoder;
    static void _encoderISR();
    void _updateEncoder();
    byte _pinEncoder;
    volatile long _encoder;
    volatile unsigned long _encoderTotal = 8192; // Inicializado a un numero grande para que nunca sea negativo
    bool _forward;
};

#endif