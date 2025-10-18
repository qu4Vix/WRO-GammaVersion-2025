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



#include "Encoder.h"

Encoder* Encoder::_sEncoder = 0;

Encoder::Encoder(byte pinEncoder) {
    _pinEncoder = pinEncoder;
    _sEncoder = this;
}

void Encoder::Attach(byte mode) {
    if ((mode < 1) || (mode > 3)) { // mode has to be between 1 and 3
        mode == CHANGE;
    }
    pinMode(_pinEncoder, INPUT);
    attachInterrupt(digitalPinToInterrupt(_pinEncoder), _encoderISR, mode);
}

void Encoder::_encoderISR() {
    _sEncoder->_updateEncoder();
}

void Encoder::_updateEncoder() {
    if (_forward) {
        _encoder++;
        _encoderTotal++;
    } else {
        _encoder--;
        _encoderTotal--;
    }
}

unsigned long Encoder::GetEncoder() {
    return _encoderTotal;
}

long Encoder::GetEncoderInterval() {
    long _encoderInterval = _encoder;
    _encoder = 0;
    return _encoderInterval;
}

void Encoder::SetMotionDirection(bool forward) {
    _forward = forward;
}