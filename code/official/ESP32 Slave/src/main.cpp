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
 * This is the code for the "Slave" ESP32 of our robot, which it is in charge of combining
 * all the different data from several sensors (camera, encoder...) and send it to the
 * main ESP32 board (the "Master") to be processed and converted into the right movements
 * of the robot.
 * 
****************************************************/



// ***** INCLUDING THE LIBRARIES *****

#include <Arduino.h>
#include <Motor.h>
#include <CServo.h>
#include <Encoder.h>
#include <HuskyLens.h> // Software serial was not found so commented on library file (max and min changed for _max and _min)
//#include "credentials.h"
#include "pinAssignments.h"
#include <rom/rtc.h>



// ***** ESTABLISHING DEFINES *****

#define ENABLE_WIFI false         // WIFI IS NOT USED ON THIS BOARD
#define ROUND_NUMBER 2            // Change this to 1 for round Open Challenge and 2 for round Obstacle Challenge

#define voltageReductionRatio 3.14 // This is the ratio between the real battery voltage and the voltage read by the ADC pin, due to the voltage divider we use since the battery voltage is higher than 3.3V - the maximum voltage the ADC pin can read. 4700 and 2200 resistors are used for the voltage divider.
#define ADC_CONVERSION_FACTOR 1240 // This is the factor to convert the voltage into ADC reading, since the ADC gives a value between 0 and 4095 corresponding to a voltage between 0 and 3.3V
#define totalConversionFactor voltageReductionRatio / ADC_CONVERSION_FACTOR // This is just to simplify the code

// Everytime we see a #if ROUND_NUMBER == 2 this is made to avoid running that part of the code when we are in Open Challenge and only run it in Obstacle Challenge
#if ROUND_NUMBER == 2 
#define TAMANO_MINIMO_ESQUIVE 20  // This is not used right now, but it is kept just in case we need it
#define ALTURA_MINIMA_ESQUIVE 40  // This is not used right now, but it is kept just in case we need it
#define MinRatioForBlock 1.5        // THIS IS THE MINIMUM HEIGH/WIDH RATIO NEEDED TO DETECT A BLOCK. This is part of a system to avoid detecting the orange line as a red block
#endif



// ***** DECLARING VARIABLES, OBJECTS AND FUNCTIONS *****

hw_timer_t* timerHandler;
HardwareSerial commSerial(1);
Motor mimotor(pinPWM, pinDir1, pinDir2, pinEn, 0.25, 1);
CServo miservo(pinServo);
CServo miservoCam(pinServoCam);
Encoder miencoder(pinEncoder_DT);
HUSKYLENS Husky;
HUSKYLENSResult fHusky;

volatile int speed;
int objectiveSpeed;

void IRAM_ATTR onTimer();
void receiveData();
void sendEncoder(uint32_t encoder);
void sendVoltage(uint8_t batteryLevel);
void sendResetReason();
void updateBattery();
void calculateNearestBlockMultipleDetectionsAndSendCamera ();
#if ROUND_NUMBER == 2
void sendCamera(uint8_t signature, uint16_t x, uint16_t y);
#endif



// ***** SETUP CODE *****
// This code will only run once, just after turning on the ESP32

void setup() {

  // Initializing code for the Serial, this is only used for debug and not during the match
  Serial.begin(115200);
  Serial.println("Starting");

  // A function of the pinAssigments.h include
  setPinModes();

  digitalWrite(pinLED_Azul, HIGH);

  // Initializing code for the  I2C, this is the connection to the camera
  Wire.begin(pinCamSDA, pinCamSCL);
  while (!Husky.begin(Wire)) delay(100);
  if (Husky.begin(Wire) == 1) Serial.println("HuskyLens connected");

  Husky.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);

  // Initializing the Communication Serial, this is how the Slave communicates to the Master
  commSerial.begin(1000000, SERIAL_8N1, pinTX, pinRX);

  // WIFI IS NOT USED
  #if ENABLE_WIFI == true
  #endif
  
  delay(5000);

  // Initializing time managment
  // The timer 0 is intialised to count up with a divider of 80, so that it clocks at 80MHz/8 = 1MHz
  timerHandler = timerBegin(0, 80, true);
  timerAttachInterrupt(timerHandler, &onTimer, false);
  // Trigger timer interrupt every 32000 clock cycles (32ms)
  timerAlarmWrite(timerHandler, 32000, true);

  // Initializing the encoder, motor and servo
  miencoder.Attach(CHANGE);
  mimotor.Init();
  miservo.BeginPWM();
  miservo.Attach();
  miservo.MoveSteeringServo(0);
  miservoCam.Attach(600, 2400);
  miservoCam.MoveServo(90);
  digitalWrite(pinLED_Azul, LOW);
  timerAlarmEnable(timerHandler);
}



// ***** LOOP CODE *****
// This code runs repeatedly

void loop() {
    
  // This receives data from the master, as the Serial Communication master-slave is a two way communication
  while (commSerial.available())
  {
    receiveData();
  }

  // Sends to the motor the right speed
  static uint32_t prev_ms_speed;
  if (millis() > prev_ms_speed) {
    mimotor.SetSpeed(speed, objectiveSpeed);
    prev_ms_speed = millis() + 32;
  }

  // Sends the battery level
  static uint32_t prev_ms_bat = millis();
  if (millis() > prev_ms_bat) {
    updateBattery();
    prev_ms_bat = millis() + 500;
  }

  // Sends the measurements of the encoder
  static uint32_t prev_ms_encoder = millis();
  if (millis() > prev_ms_encoder) {
    sendEncoder(miencoder.GetEncoder());
    prev_ms_encoder = millis() + 32;
  }

  // Sends the position and type of the nearest block detected by the camera
  #if ROUND_NUMBER == 2
  static uint32_t prev_ms_camera = millis();
  if (millis() > prev_ms_camera) {
    Husky.requestBlocksLearned();
    calculateNearestBlockMultipleDetectionsAndSendCamera();
    prev_ms_camera = millis() + 100;
  }
  #endif
}



// ***** FUNCTIONS *****

void IRAM_ATTR onTimer() {
  speed = miencoder.GetEncoderInterval();
}

// Sends back to the master the encoder information using our own communication protocol
void sendEncoder(uint32_t encoder) {
  uint8_t encoderBuffer[4];
  for (uint8_t i; i<4; i++) {
    encoderBuffer[i] = ((encoder>>(8*i)) & 0xff);
  }
  commSerial.write(7);
  commSerial.write(encoderBuffer, 4);
}

// Sends back to the master the battery level
void sendVoltage(uint8_t batteryLevel) {
  commSerial.write(6);
  commSerial.write(batteryLevel);
}

// Sends back the x and y position of the detected block and also its signature (1 or 2 depending on the color)
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

// Sends the reset reason back to the master
void sendResetReason() {
  commSerial.write(4);
  commSerial.write(rtc_get_reset_reason(0));
  commSerial.write(rtc_get_reset_reason(1));
}

// This function is in charge of receiving the data sent to the slave by the master. This consists of the speed we want to move the robot and the angle for the servo
void receiveData() {
  uint8_t firstByte;
  commSerial.readBytes(&firstByte, 1);
  if (firstByte == 1) // This reads the speed information
  {
    uint8_t _velocity;
    commSerial.readBytes(&_velocity, 1);
    uint8_t _speed = (_velocity >> 1);
    if (_velocity & 0x01) {
      objectiveSpeed = -_speed; // if the LSB is true, the speed is negative
      miencoder.SetMotionDirection(false); // set driving direction in the encoder to backwards (not forward)
    } else {
      objectiveSpeed = _speed; // otherwise it is positive
      miencoder.SetMotionDirection(true); // set driving direction in the encoder to forwards
    }
  } else 
  if (firstByte == 2) // This reads the angle information
  {
    uint8_t _angleByte;
    commSerial.readBytes(&_angleByte, 1);
    int _angle = _angleByte - 90;
    miservo.MoveSteeringServo(_angle);
  } else
  if (firstByte == 3) // This reads a the angle information for the camera servo
  {
    uint8_t _angleByte;
    commSerial.readBytes(&_angleByte, 1);
    miservoCam.MoveServo(_angleByte);
  }
}

// Reads the current voltage of the battery
void updateBattery() {
  uint16_t voltage = analogRead(pinVoltage) * totalConversionFactor; // battery voltage in V;
  if (voltage >= 8) {
    // HIGH LEVEL
    sendVoltage(1);
  } else if (voltage >= 7.6) {
    // MEDIUM LEVEL
    sendVoltage(2);
  } else {
    // LOW LEVEL
    sendVoltage(3);
  }
}

// This function looks for the nearest block and sends back its information (position and ID). This code is made to avoid confusion with the orange line of the board or the parking walls 
#if ROUND_NUMBER == 2
void calculateNearestBlockMultipleDetectionsAndSendCamera (){
  int16_t numberOfBlocks = Husky.countBlocksLearned();

  int16_t blocksIndexNumber[numberOfBlocks];
  uint16_t blocksHeight[numberOfBlocks];
  uint16_t blocksWidth[numberOfBlocks];
  uint16_t blocksSurface[numberOfBlocks];

  int16_t maxSurfaceIndex = -1;
  int16_t maxSurface = -1;

  int ourID; 

  // Because the camera detects all blocks and doesn't tell us which is closer, we first read all of them and put their information in this three arrays
  for (int i = 0; i < numberOfBlocks; i++)
  {
    blocksIndexNumber[i] = i;
    
    fHusky = Husky.getBlockLearned(i);
    blocksHeight[i] = fHusky.height;
    blocksWidth[i] = fHusky.width;
    blocksSurface[i] = fHusky.height * fHusky.width;
  }

  // Now we search which block is the highest (closer to the camera). That block has to be taller than it is wide because otherwise it is probably a line, as lines will often be wider than they are tall.
  for (int i = 0; i < numberOfBlocks; i++)
  {
    if((blocksSurface[i] > maxSurface) && (((double)blocksHeight[i]/(double)blocksWidth[i]) > MinRatioForBlock))
    {
      maxSurface = blocksSurface[i];
      maxSurfaceIndex = blocksIndexNumber[i];

    } 
  }

  // This conditional avoids having problems when the camera does not detect any valid object
  if((maxSurfaceIndex == -1 ) || (maxSurface == -1)) 
  {
  }else{
    fHusky = Husky.getBlockLearned(maxSurfaceIndex);
    
    if((fHusky.ID >= 1) && (fHusky.ID <=4))
    {
      ourID = 1;
    } else {
      ourID = 2;
    }
    
    sendCamera(ourID, ((315*fHusky.xCenter)/320), ((207*fHusky.yCenter)/240));  // We use this rule of three to adapt it to an older version of this code
  }
}
#endif