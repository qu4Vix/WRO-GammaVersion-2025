/***************************************************
 * 
 * THE MAIN REPOSITORY CAN BE FOUND AT https://github.com/qu4Vix/WRO-GammaVersion-2025
 * 
 * This code is under a GPL-3.0 license
 * 
****************************************************/

/***************************************************
 * 
 * This is the code for the "Slave" ESP32 of our robot, which it is in charge of combining
 * all the different data from several sensors (camera, encoder...) and send it to the
 * main ESP32 board (the "Master") to be processed and translated into the correct moving
 * of the robot.
 * 
****************************************************/

// ***** INCLUDING THE LIBRARIES *****
#include <Arduino.h>
#include <Motor.h>
#include <CServo.h>
#include <Encoder.h>
#include <HuskyLens.h> // Software serial was not found so commented on library file (max and min changed for _max and _min)
#include "credentials.h"
#include "pinAssignments.h"
#include <rom/rtc.h>

#define ENABLE_WIFI false // WIFI IS NOT USED ON THIS BOARD
#define ROUND_NUMBER 2  // Change this to 1 for round Open Challenge and 2 for round Obstacle Challenge

hw_timer_t* timerHandler;

HardwareSerial commSerial(1);
Motor mimotor(pinPWM, pinDir1, pinDir2, pinEn, 0.25, 1);
CServo miservo(pinServo);
Encoder miencoder(pinEncoder_DT);

HUSKYLENS Husky;
HUSKYLENSResult fHusky;

#if ROUND_NUMBER == 2

#define TAMANO_MINIMO_ESQUIVE 20
#define ALTURA_MINIMA_ESQUIVE 40

#endif

volatile int speed;
int objectiveSpeed;


void IRAM_ATTR onTimer();

void receiveData();
void sendEncoder(uint32_t encoder);
void sendTension(uint8_t batteryLevel);
void sendResetReason();
void actualizarBateria();
void calculateNearestBlockAndSendCamera ();
// Battery levels
// 8.4V - 3600
// 8V - 3300
// 7.6V - 3100

#if ROUND_NUMBER == 2
void sendCamera(uint8_t signature, uint16_t x, uint16_t y);
#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Empezando");

  Wire.begin(18,19);
  while(!Husky.begin(Wire)) delay(100);
  
  setPinModes();

  commSerial.begin(1000000, SERIAL_8N1, pinTX, pinRX);

  #if ENABLE_WIFI == true
  
  #endif
  
  delay(5000);

  //sendResetReason();

  timerHandler = timerBegin(0, 80, true);
  timerAttachInterrupt(timerHandler, &onTimer, false);
  timerAlarmWrite(timerHandler, 32000, true);
  
  #if ROUND_NUMBER == 2
  
  #endif

  miencoder.Attach(CHANGE);
  mimotor.Init();
  miservo.BeginPWM();
  miservo.Attach();
  miservo.MoveServo(0);
  timerAlarmEnable(timerHandler);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while (commSerial.available())
  {
    receiveData();
  }

  static uint32_t prev_ms_speed;
  if (millis() > prev_ms_speed) {
    mimotor.SetSpeed(speed, objectiveSpeed);
    prev_ms_speed = millis() + 32;
  }

  static uint32_t prev_ms_bat = millis();
  if (millis() > prev_ms_bat) {
    actualizarBateria();
    prev_ms_bat = millis() + 500;
  }

  static uint32_t prev_ms_encoder = millis();
  if (millis() > prev_ms_encoder) {
    sendEncoder(miencoder.GetEncoder());
    prev_ms_encoder = millis() + 32;
  }

  #if ROUND_NUMBER == 2
  static uint32_t prev_ms_camera = millis();
  if (millis() > prev_ms_camera) {
    calculateNearestBlockAndSendCamera();
    prev_ms_camera = millis() + 100;
  }
  #endif
}

void IRAM_ATTR onTimer() {
  speed = miencoder.GetEncoderInterval();
}

void sendEncoder(uint32_t encoder) {
  uint8_t encoderBuffer[4];
  for (uint8_t i; i<4; i++) {
    encoderBuffer[i] = ((encoder>>(8*i)) & 0xff);
  }
  commSerial.write(7);
  commSerial.write(encoderBuffer, 4);
}

void sendTension(uint8_t batteryLevel) {
  commSerial.write(6);
  commSerial.write(batteryLevel);
}

//Adaptar para la husky
#if ROUND_NUMBER == 2
void sendCamera(uint8_t signature, uint16_t x, uint16_t y) {
  uint8_t _x = uint8_t(x/2);
  uint8_t _y = uint8_t(y/2);
  commSerial.write(5);
  commSerial.write(signature);
  commSerial.write(_x);
  commSerial.write(_y);
}
#endif

void sendResetReason() {
  commSerial.write(4);
  commSerial.write(rtc_get_reset_reason(0));
  commSerial.write(rtc_get_reset_reason(1));
}

void receiveData() {
  uint8_t firstByte;
  commSerial.readBytes(&firstByte, 1);
  if (firstByte == 1)
  {
    uint8_t _velocity;
    commSerial.readBytes(&_velocity, 1);
    uint8_t _speed = (_velocity >> 1)<<1;
    int speed = (_velocity - _speed) ? -_speed : _speed;
    objectiveSpeed = speed;
  } else 
  if (firstByte == 2)
  {
    uint8_t _angleByte;
    commSerial.readBytes(&_angleByte, 1);
    int _angle = _angleByte - 90;
    miservo.MoveServo(_angle);
  }
}

void actualizarBateria() {
  uint16_t tension = analogRead(pinTension);
  if (tension >= 3300) {
    // HIGH LEVEL
    sendTension(1);
  } else if (tension >= 3100) {
    // MEDIUM LEVEL
    sendTension(2);
  } else {
    // LOW LEVEL
    sendTension(3);
  }
}

#if ROUND_NUMBER == 2
void calculateNearestBlockAndSendCamera (){
  int16_t numberOfBlocks = Husky.countBlocksLearned();

  int16_t blocksIndexNumber[numberOfBlocks];
  uint16_t blocksHeight[numberOfBlocks];

  int16_t maxHeightIndex;
  uint16_t maxHeight = 0;

  for (int i = 0; i < numberOfBlocks; i++)
  {
    blocksIndexNumber[i] = i;
    
    fHusky = Husky.getBlockLearned(i);
    blocksHeight[i] = fHusky.height;
  }

  for (int i = 0; i < numberOfBlocks; i++)
  {
    if(blocksHeight[i] > maxHeight)
    {
      maxHeight = blocksHeight[i];
      maxHeightIndex = blocksIndexNumber[i];
    } 
  }

  fHusky = Husky.getBlockLearned(maxHeightIndex);
  sendCamera(fHusky.ID, ((315*fHusky.xCenter)/320), ((207*fHusky.yCenter)/240));
}
#endif