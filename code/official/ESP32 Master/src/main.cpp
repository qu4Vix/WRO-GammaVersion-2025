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
 * This is the code for the "Main" ESP32 of our robot, which it is responsible for controlling
 * some sensors and to make all the decisions.
 * 
 * THIS VERSION OF THE ESP32 MASTER CODE IS ONLY FOR THE OPEN CHALLENGE
 * For the Obstacle Challenge use "ESP32 MASTER OBSTACLES"
 *  
****************************************************/



// ***** INCLUDING THE LIBRARIES *****

#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include <melody_factory.h>
#include <melody_player.h>
#include "pinAssignments.h"
#include <rom/rtc.h>
#include <esp_task_wdt.h>



// ***** ESTABLISHING DEFINES *****

// Practice mode (Button desabled) for easy launches while testing
#define PRACTICE_MODE true

// Enables wifi functions when true
#define ENABLE_WIFI false       // WIFI IS NOT USED ON THIS BOARD DURING THE MATCH, only for debug purpose
#define ENABLE_TELEMETRY true

// Speeds
#define StartSpeed 4
#define CruisiereSpeed 10
#define NormalSpeed 5

// Servo and direction variables
#define servoKP 2.5
#define servoKD 0

// position PID controller variables
#define positionKP 0.1
#define positionKD 1



// ***** INITIALIZING VARIABLES, OBJECTS AND FUNCTIONS *****
int prev_setAngle;
int actual_directionError;
int prev_directionError;
float objectiveDirection;

// Battery level variable
uint8_t bateria;

// Conversion between mm and encoder counts
#define MMperEncoder 1.41

// List of possible states for the car
enum e {
  Inicio,
  Recto,
  Final,
  Prueba
};
uint8_t estado = e::Inicio;

// Size of the map (mm)
#define mapSize 3000

// Journey variables
uint8_t giros = 0;
uint8_t tramo = 0;
int8_t turnSense = 0;       // We don't know it at the beginning
int8_t motionDirection = 0; // The car doesn't move at the beginning

// Encoder variables
int32_t encoderMeasurement;
int32_t prev_encoderMeasurement;

// Lidar measurement variables
uint16_t distances[360];
static uint16_t distancesMillis[360];

// Car position on the map (origin at the bottom-left corner)
double xPosition = 0;   // x position of the car (increases to the right)
double yPosition = 0;   // y position of the car (increases upwards)

// Position PID controller variables
int objectivePosition = 0;
int positionError;
int prev_positionError;
bool fixXposition = true;
bool fixInverted = true;

// Object declarations
MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;
HardwareSerial commSerial(1);
TaskHandle_t Task1;
HardwareSerial teleSerial(0);

// Music
MelodyPlayer player(pinBuzzer, HIGH);
const uint8_t nNotes = 45;
String notes1[nNotes] = 
{
  "A4", "E5", "SILENCE",    "A4", "E5", "SILENCE",    "B4", "E5", "SILENCE",    "B4", "E5", "SILENCE",
  "C4", "E5", "SILENCE",    "C4", "E5", "SILENCE",    "D4", "E5", "SILENCE",    "D4", "E5", "B4",
  "A4", "E5", "SILENCE",    "A4", "E5", "SILENCE",    "B4", "E5", "SILENCE",    "B4", "E5", "SILENCE",
  "C4", "E5", "SILENCE",    "C4", "E5", "SILENCE",    "D4", "E5", "SILENCE"
};
const uint16_t timeUnit = 625;
// Create a melody
Melody melody1 = MelodyFactory.load("Cornfield chase", timeUnit, notes1, nNotes);
String notes2[9] = {"D4", "D4", "D4", "D4", "E5", "E5", "E5", "E5", "B4"};
Melody melody2 = MelodyFactory.load("Cornfield chase 2", 156, notes2, 9);
String notes3[60] =
{
  "A4", "C4", "E5", "A5",   "A4", "C4", "E5", "A5",   "A4", "C4", "E5", "A5",
  "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",
  "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",
  "A4", "C4", "E5", "A5",   "G3", "B4", "E5", "F5",   "A4", "C4", "E5", "A5",
  "B4", "D4", "E5", "B5",   "A4", "C4", "E5", "A5",   "G3", "B4", "E5", "F5"
};
Melody melody3 = MelodyFactory.load("Cornfield chase 3", 156, notes3, 60);
String notes4[18] =
{
  "A5", "E6", "A5",   "A5", "E6", "A5",   "B5", "E6", "B5",   "B5", "E6", "B5",   "C5", "E6", "C5",   "C5", "E6", "C5"
};
Melody melody4 = MelodyFactory.load("Cornfield chase 4", 625, notes4, 18);

u_int16_t distancia90;
u_int16_t distancia270;

// Declarations
int directionError(int bearing, int target);  // calculate the error in the direction

// ESP32 intercommunication functions 
void setSpeed(int speed);             // set the car's speed
void setSteering(int angle);          // set the angle of the servo
void receiveData();                   // receive data from the serial
void manageTension(uint8_t tension);  // turn a led on depending on the tension received

// LIDAR management variables
uint16_t getIndex(float angle);           // get the index in the distances array for an angle given
uint16_t readDistance(uint16_t angle);    // Angle from 0 to 359
void LidarTaskCode(void * pvParameters);  // Create code for the task which manages the LIDAR

// Send a piece of data to the telemetry esp32
void enviarDato(byte* pointer, int8_t size);
void sendPacket(byte packetType, byte* packet);

// Functions for the management of the car's position
void iteratePositionPID();    // invoke an iteration of the pid controller for the position
void turn();                  // turn
void setXcoord(uint16_t i);   // set the coordinate x axis
void setYcoord(uint16_t f);   // set the coordinate y axis
void decideTurn();            // detect the sense of turn
void checkTurn();             // check wether you have to turn or not



// ***** SETUP CODE *****
// This code will only run once, just after turning on the ESP32

void setup() {
  
  //THIS IS ONLY USED FOR DEBUG PURPOSES AND NOT DURING THE MATCH
  #if ENABLE_TELEMETRY == true
    // Begin telemetry serial
    teleSerial.begin(1000000, SERIAL_8N1, telemetriaRX, telemetriaTX);
  #else
    // Begin serial
    Serial.begin(115200);
  #endif

  // Begin ESP32 Master-Slave intercommunication serial
  commSerial.begin(1000000, SERIAL_8N1, pinRX, pinTX);

  // Set all the pin modes
  setPinModes();
  digitalWrite(pinBuzzer, HIGH);

  // WIFI IS NOT USED
  #if ENABLE_WIFI == true
  #endif

  // Configure the IMU MPU
  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();

  delay(500);

  // Begin the lidar and check its health
  lidar.begin(lidarSerial);
  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  Serial.println("info: " + String(health.status) +", " + String(health.error_code));
  lidar.startScan();

  esp_task_wdt_deinit();

  // Asign LIDAR Task to core 0
  xTaskCreatePinnedToCore(
    LidarTaskCode,
    "Task1",
    100000,
    NULL,
    10,
    &Task1,
    0);

  delay(500);
  digitalWrite(pinBuzzer, LOW);

  // Start LIDAR's motor rotating at max allowed speed
  analogWrite(pinLIDAR_motor, 255);
  delay(500);

  // Wait until y coordinate is calculated
  while (readDistance(0) == 0) delay(100);
  setYcoord(readDistance(0));
  delay(500);

  #if PRACTICE_MODE == true
  // Receive data from the intercommunication serial
  while (commSerial.available())
  {
    receiveData();
  }
  prev_encoderMeasurement = encoderMeasurement;
  #else
  // Waits until the start button is pressed
  digitalWrite(pinLED_verde, HIGH);
  while (digitalRead(pinBoton)) {
    while (commSerial.available())
    {
      receiveData();
    }
  }
  prev_encoderMeasurement = encoderMeasurement;
  digitalWrite(pinLED_verde, LOW);
  #endif
  delay(1000);

  // Starts driving (set a speed to the car and initialize the IMU MPU)
  setSpeed(StartSpeed);
  mimpu.measureFirstMillis();
}



// ***** LOOP CODE *****
// This code runs repeatedly

void loop() {

  // Receive data from the intercommunication serial
  while (commSerial.available())
  {
    receiveData();
  }

  // Update IMU MPU's angle
  mimpu.UpdateAngle();

  // Repeat position functions every 32ms
  static uint32_t prev_ms_position = millis();
  if (millis() > prev_ms_position) {
    if (encoderMeasurement != prev_encoderMeasurement) {
      // Calculate the increment in position and add it
      double dy = (encoderMeasurement - prev_encoderMeasurement) * cos(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      double dx = (encoderMeasurement - prev_encoderMeasurement) * sin(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      prev_encoderMeasurement = encoderMeasurement;
      xPosition -= dx; // x -> + right - left
      yPosition += dy;
      iteratePositionPID();
    }
    prev_ms_position = millis() + 32;
  }

  // Send telemetry every 100ms, ONLY USED FOR DEBUG PURPOSES AND NOT DURING THE MATCH
  #if ENABLE_TELEMETRY == true
  static uint32_t prev_ms_tele = millis();
  if (millis() > prev_ms_tele + 100)
  {
    /*Telemetry format
    |StartTX             |PacketType |Data|
      0xAA,0xAA,0xAA,0xAA,0x--,0x--...0x--
    */
    // WE SEND PACKET TYPE 4 DISTANCES
    /*for (int i = 0; i<4; i++) {
      teleSerial.write(0xAA);
    }
    teleSerial.write(04);
    uint16_t zi=0;
    while (zi < 360)
    {
        teleSerial.write(distances[zi]>>8);
        teleSerial.write(distances[zi]&0x00ff);
        zi++;
    }

    for (int i = 0; i<4; i++) {
      teleSerial.write(0xAA);
    }
    teleSerial.write(05);
    uint16_t wi=0;
    while (wi < 360)
    {
        uint16_t distanceAge =  millis() - distancesMillis[zi];
        teleSerial.write(distanceAge>>8);
        teleSerial.write(distanceAge&0x00ff);
        wi++;
    }*/
   
    // WE SEND PACKET TYPE 3 MEDIUM QUALITY
    /*
    for (int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(03);
    uint16_t pi=0;
    while (pi < 360){
        teleSerial.write(distancesArray[1][pi]);
        pi++;
    }*/

    // WE SEND PACKET TYPE 5 GENERAL INFORMATION
                /*
            -- X Position, 8 bytes
            -- Y Position, 8 bytes
            -- X Objective position, 8 bytes
            -- Y Objective position, 8 bytes
            -- Encoder, 32 uint32
            -- State, 8 bits uint
            -- Battery, 8 bits uint
            -- Angle, 16 float
            -- Objective angle, 16 float
            -- Camera signature 1 detected, 1 byte
            -- Camera signature 1 x, 8 bits
            -- Camera signature 1 y, 8 bits
            -- Camera signature 2 detected, 1 byte
            -- Camera signature 2 x, 8 bits
            -- Camera signature 2 y, 8 bits  
            
            |XXXX|YYYY|MMMM|NNNN|QQQQ|W|E|RRRR|TTTT|U|I|O|A|S|D
             0000 0000 0111 1111 1112 2 2 2222 2223 3 3 3 3 3 3
             1234 5678 9012 3456 7890 1 2 3456 7890 1 2 3 4 5 6
            */
    // We send the start header of the packet
    for (int i = 0; i<4; i++) {
      teleSerial.write(0xAA);
    }

    // We send the packet
    teleSerial.write(byte(06));
    unsigned long time = millis();
    long posXLong = long(xPosition);
    long posYLong = long(yPosition);
    long posXObjLong = long(objectivePosition);
    long posYObjLong = (turnSense==-1)?1:(turnSense==1)?2:0;
    long anguloLong = long(mimpu.GetAngle());
    long anguloObjLong = long(objectiveDirection);
    enviarDato((byte*)&time,sizeof(time));
    enviarDato((byte*)&posXLong,sizeof(posXLong));
    enviarDato((byte*)&posYLong,sizeof(posYLong));
    enviarDato((byte*)&posXObjLong,sizeof(posXObjLong));
    enviarDato((byte*)&posYObjLong,sizeof(posYObjLong));
    enviarDato((byte*)&encoderMeasurement,sizeof(encoderMeasurement));
    enviarDato((byte*)&estado,sizeof(estado));
    enviarDato((byte*)&bateria,sizeof(bateria));
    enviarDato((byte*)&anguloLong,sizeof(anguloLong));
    enviarDato((byte*)&anguloObjLong,sizeof(anguloObjLong));
    enviarDato((byte*)&tramo,sizeof(tramo));
    enviarDato((byte*)&distancia90,sizeof(distancia90));
    enviarDato((byte*)&distancia270,sizeof(distancia270));
    
    prev_ms_tele = millis();
  }
  #endif

  // Check turn every 50ms
  static uint32_t prev_ms_turn = millis();
  if (millis() > prev_ms_turn) {
    checkTurn();
    prev_ms_turn = millis() + 50;
  }

  // Repeat direction PID iterations every 20ms
  // ONLY GOD KNOWS WHAT IS HAPPENING HERE
  static uint32_t prev_ms_direction = millis();
  if (millis() > prev_ms_direction) {
    actual_directionError = constrain(directionError(mimpu.GetAngle(), objectiveDirection), -127, 127);
    int _setAngle = servoKP * actual_directionError + servoKD * (actual_directionError - prev_directionError);
    if(_setAngle != prev_setAngle) {
      setSteering(_setAngle*motionDirection);
      prev_setAngle = _setAngle;
    }
    prev_directionError = actual_directionError;
    prev_ms_direction = millis() + 20;
  }

  // State machine depending of the position and situation of the robot during the match
  switch (estado)
  {
  case e::Inicio:   // Start position
    if (yPosition >= 1500) {
      digitalWrite(pinLED_rojo, HIGH);
      decideTurn();
      if (yPosition >= 2400) {
        setSpeed(0);
        if (turnSense != 0) {
          digitalWrite(pinLED_rojo, LOW);
          digitalWrite(pinLED_verde, HIGH);
          digitalWrite(pinBuzzer, HIGH);
          setXcoord(readDistance(270));
          objectivePosition = xPosition;
          vTaskDelete(Task1);
          analogWrite(pinLIDAR_motor, 0);
          estado = e::Recto;
          setSpeed(NormalSpeed);
          digitalWrite(pinBuzzer, LOW);
        }
      }
    }
  break;
  case e::Recto:
    if (giros == 12) {
      estado = e::Final;
    }
  break;
  case e::Final:
    if (yPosition >= 1200) {
      setSpeed(0);
    }
  break;
  //Caso prueba para probar el encoder y calcular MMperEncoder
  case e::Prueba:
    if (yPosition >= 3000) setSpeed(0);
  }
}

// Definitions

int directionError(int bearing, int target) {
  int error = target - bearing;
  return error;
}

void setSpeed(int speed) {
  speed = constrain(speed, -100, 100);
  uint8_t _speed = abs(speed) << 1;
  if (speed >= 0) {
    motionDirection = 1;
  } else {
    motionDirection = -1;
    _speed = _speed | 0b01;
  }
  commSerial.write(1);
  commSerial.write(_speed);
}

void setSteering(int angle) {
  angle = constrain(angle, -90, 90);
  uint8_t _angle = angle + 90;
  commSerial.write(2);
  commSerial.write(_angle);
}

// Receive the data from commSerial - the Serial which connects to the ESP32 Slave
void receiveData() {
  //Receive the first byte (the header)
  uint8_t firstByte;
  commSerial.readBytes(&firstByte, 1);
  // Header 7 gives the current encoder measurement
  if (firstByte == 7) {
    uint8_t bytes[4];
    commSerial.readBytes(bytes, 4);
    encoderMeasurement = 0;
    for (uint8_t iteration; iteration < 4; iteration++) {
      encoderMeasurement = encoderMeasurement | bytes[iteration] << (8*iteration);
    }
  } else
  // Header 6 gives the current state of the battery (the current voltage level: 1, 2 or 3)
  if (firstByte == 6) {
    uint8_t tensionValue;
    commSerial.readBytes(&tensionValue, 1);
    bateria = tensionValue;
    manageTension(tensionValue);
  } /*else
  // Header 4 gives the reset reason of the ESP32 Slave and sends it alongside the Master's trought telemetry
  if (firstByte == 4) {
    uint8_t slaveResetReason[2];
    commSerial.readBytes(slaveResetReason, 2);
    for (int i = 0; i<4; i++) {   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(01);
    for (int i=0; i<6; i++) {
        teleSerial.write(1);
    }
    teleSerial.write(rtc_get_reset_reason(0));
    teleSerial.write(rtc_get_reset_reason(1));
    teleSerial.write(slaveResetReason[0]);
    teleSerial.write(slaveResetReason[1]);
  }*/
}

void manageTension(uint8_t tension) {
  if (tension == 1) {
    digitalWrite(pinLED_batAmarillo, LOW);
    digitalWrite(pinLED_batRojo, LOW);
    digitalWrite(pinLED_batVerde, HIGH);
  } else if (tension == 2) {
    digitalWrite(pinLED_batVerde, LOW);
    digitalWrite(pinLED_batRojo, LOW);
    digitalWrite(pinLED_batAmarillo, HIGH);
  } else if (tension == 3) {
    digitalWrite(pinLED_batVerde, LOW);
    digitalWrite(pinLED_batAmarillo, LOW);
    digitalWrite(pinLED_batRojo, HIGH);
  }
}

uint16_t getIndex(float angle) {
  if (angle >= 359.5) return 0;
  float error = angle - uint16_t(angle);
  if (error < 0.5) {
    return uint16_t(angle);
  } else {
    return uint16_t(angle + 1);
  }
}

#define numberOfMeasures 5
// Angle from 0 to 359
uint16_t readDistance(uint16_t angle) {
  int index = -numberOfMeasures;
  int validIndex = 0;
  int validMeasures[2*numberOfMeasures];

  int index2 = -numberOfMeasures;
  uint16_t f_distances[360];
  uint16_t f_distancesMillis[360];
  
  
  for (int i = 0; i < 360; i++) {
    f_distances[i] = distances[i];
    f_distancesMillis[i] = distancesMillis[i];
  }
  

  while (index < numberOfMeasures) {
    // work out the resultant index for the distances array
    int resAngle = angle + index;
    if (resAngle < 0) resAngle += 360;
    // check whether the measurement is non zero and new and store it into the next place of the array
    if (f_distances[resAngle] == 0) {}
    else if ((millis() - f_distancesMillis[resAngle]) < 800)
    {
      validMeasures[validIndex] = resAngle;
      validIndex++;
    }

    index++;
  }
  // search for two consecutive measurements that are similar
  for (int arrayIndex = 1; arrayIndex < validIndex; arrayIndex++) {
    if (abs(f_distances[validMeasures[arrayIndex]] - f_distances[validMeasures[arrayIndex - 1]]) < 50) {
      return _min(f_distances[validMeasures[arrayIndex]], f_distances[validMeasures[arrayIndex - 1]]);
    }
  }
  // if the search fails return 0
  return 0;
}

// Create code for the task which manages the lidar
void LidarTaskCode(void * pvParameters) {
  
  for (;;) {
    vTaskDelay(1);
    if (IS_OK(lidar.waitPoint())) {
      // record data
      uint16_t distance = uint16_t(lidar.getCurrentPoint().distance); //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //angle value in degrees

      // obtain the index associated with the angle and store in the array
      uint16_t index = getIndex(angle);

      if (distance > 100 && distance < mapSize)
      { 
        //bool  startBit = lidar.getCurrentPoint().startBit; //whether this point belongs to a new scan
        //byte quality = lidar.getCurrentPoint().quality;

        distances[index] = distance;
        distancesMillis[index] = millis();
      } else {
        distances[index] = 0;
        distancesMillis[index] = 0;
      }
    }
  }
}

void iteratePositionPID() {
  prev_positionError = positionError;
  if (fixXposition) {
    positionError = directionError(xPosition, objectivePosition);
  } else {
    positionError = directionError(yPosition, objectivePosition);
  }
  objectiveDirection = constrain(positionKP * positionError + positionKD * (positionError - prev_positionError), -90, 90) * motionDirection;
  if (fixInverted) objectiveDirection = -objectiveDirection;
  objectiveDirection += 90 * giros * turnSense;
}

void turn() {
  switch ((tramo+1) * turnSense)
  {
  case -1:
    objectivePosition = 2700;
    fixInverted = false;
    tramo = 1;
    break;
  
  case -2:
    objectivePosition = 2700;
    fixInverted = false;
    tramo = 2;
    break;

  case -3:
    objectivePosition = 300;
    fixInverted = true;
    tramo = 3;
    break;

  case -4:
    objectivePosition = 300;
    fixInverted = true;
    tramo = 0;
    break;
  
  case 1:
    objectivePosition = 2700;
    fixInverted = true;
    tramo = 1;
    break;
  
  case 2:
    objectivePosition = 400;
    fixInverted = false;
    tramo = 2;
    break;

  case 3:
    objectivePosition = 400;
    fixInverted = false;
    tramo = 3;
    break;

  case 4:
    objectivePosition = 2700;
    fixInverted = true;
    tramo = 0;
    break;
  }
  giros++;
  fixXposition = !fixXposition;
}

void setXcoord(uint16_t i) {
  xPosition = i;
}

void setYcoord(uint16_t f) {
  yPosition = mapSize - f - 150;
}

void checkTurn() {
  switch ((tramo+1) * turnSense)
  {
  case -1:
    if (yPosition >= 2700 - 200) turn();
    break;
  
  case -2:
    if (xPosition >= 2700 - 200) turn();
    break;

  case -3:
    if (yPosition <= 300 + 200) turn();
    break;

  case -4:
    if (xPosition <= 300 + 300) turn();
    break;

  case 1:
    if (yPosition >= 2700 - 300) turn();
    break;
  
  case 2:
    if (xPosition <= 300 + 300) turn();
    break;

  case 3:
    if (yPosition <= 300 + 300) turn();
    break;

  case 4:
    if (xPosition >= 2700 - 300) turn();
    break;
  }
}

void decideTurn() {
  distancia90 = readDistance(90);
  distancia270 = readDistance(270);
  if (distancia90 > distancia270 && distancia90 > 1000)
  {
    turnSense = -1;
  }
  else if (distancia270 > distancia90 && distancia270 > 1000)
  {
    turnSense = 1;
  }
  else turnSense = 0;
}

void enviarDato(byte* pointer, int8_t size) {
  int8_t posicion = size - 1;   //recorreremos la memoria desde el de mas valor hara el de menos, ya que el 
                          //receptor espera ese orde MSB
  while (posicion >= 0 ) {
    teleSerial.write(pointer[posicion]);
    posicion--;
  }
}

void sendPacket(byte packetType, byte* packet) {
  byte size;
  if (sizeof(packet) == size);
  for (int i = 0; i<4; i++) {     //Send the start of packet header
    teleSerial.write(0xAA);
  }
  teleSerial.write(packetType);   //Send the packet type
  for (int i=0; i<size; i++) {    //Send the packet data
      teleSerial.write(packet[i]);
  }
}