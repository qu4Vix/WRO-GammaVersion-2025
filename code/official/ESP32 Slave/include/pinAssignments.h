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
 * We can find here the different pin assignments.
 * 
****************************************************/

// LED pins
#define pinLED_1 34
#define pinLED_2 35

// Motor Pins (PWM + 3 Digital)
#define pinPWM 14
#define pinDir1 26
#define pinDir2 27
#define pinEn 25

// Servo pin (PWM)
#define pinServo 13

// Encoder pins
#define pinEncoder_CLK 32
#define pinEncoder_DT 33

// Interchip Communication pins (UART)
#define pinTX 4
#define pinRX 2

// Battery tension reader pin
#define pinVoltage 36

void setPinModes() {
    pinMode(pinVoltage, INPUT);
    pinMode(pinEncoder_DT, INPUT);
    pinMode(pinEncoder_CLK, INPUT);
}