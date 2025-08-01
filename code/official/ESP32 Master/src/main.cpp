#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include <melody_factory.h>
#include <melody_player.h>
//#include "credentials.h"
#include "pinAssignments.h"
#include <rom/rtc.h>
#include <esp_task_wdt.h>

// Practice mode (Button desabled) for easy launches while testing
#define PRACTICE_MODE true

// Enables wifi functions when true
#define ENABLE_WIFI false

// Servo and direction variables

#define servoKP 2.5
#define servoKD 0
int prev_setAngle;
int actual_directionError;
int prev_directionError;
float objectiveDirection;

// battery level variable
uint8_t bateria;

// conversion between mm and encoder counts
#define MMperEncoder 1.41

// List of possible states for the car
enum e {
  Inicio,
  Recto,
  DecidiendoGiro,
  PreGiro,
  Girando,
  Final,
  Prueba
};
uint8_t estado = e::Inicio;

// journey variables

uint8_t giros = 0;
uint8_t tramo = 0;
int8_t turnSense = 0;

// encoder variables

uint32_t encoderMeasurement;
uint32_t prev_encoderMeasurement;

// lidar measurement variables

uint16_t distances[360];
static uint16_t distancesMillis[360];

// car position on the map (origin at the bottom-left corner)

// x position of the car (increases to the right)
double xPosition = 0;
// y position of the car (increases upwards)
double yPosition = 0;

// position PID controller variables

#define positionKP 0.1
#define positionKD 1
int objectivePosition = 0;
int positionError;
int prev_positionError;
bool fixXposition = true;
bool fixInverted = true;

// object declarations

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
// create a melody
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


// Delcarations

// calculate the error in the direction
int directionError(int bearing, int target);

// esp32 intercommunication functions

// set the car's speed
void setSpeed(int speed);
// set the angle of the servo
void setSteering(int angle);
// receive data from the serial
void receiveData();
// turn a led on depending on the tension received
void manageTension(uint8_t tension);

// LIDAR management variables

// get the index in the distances array for an angle given
uint16_t getIndex(float angle);
// Angle from 0 to 359
uint16_t readDistance(uint16_t angle);
// Create code for the task which manages the LIDAR
void LidarTaskCode(void * pvParameters);

// send a piece of data to the telemetry esp32
void enviarDato(byte* pointer, int8_t size);
void sendPacket(byte packetType, byte* packet);

// functions for the management of the car's position

void iteratePositionPID();  // invoke an iteration of the pid controller for the position
void turn();            // turn
void setXcoord(uint16_t i);   // set the coordinate x axis
void setYcoord(uint16_t f);   // set the coordinate y axis
void decideTurn();  // detect the sense of turn
void checkTurn();   // check wether you have to turn or not

void setup() {
  // put your setup code here, to run once:

  // begin serial
  Serial.begin(115200);
  // begin telemetry serial
  teleSerial.begin(1000000, SERIAL_8N1, telemetriaRX, telemetriaTX);
  // begin esp32 intercommunication serial
  commSerial.begin(1000000, SERIAL_8N1, pinRX, pinTX);

  // set all the pin modes
  setPinModes();
  digitalWrite(pinBuzzer, HIGH);
  #if ENABLE_WIFI == true
    
  #endif
  // configure the mpu
  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();
  delay(1000);
  // begin the lidar
  lidar.begin(lidarSerial);
  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  Serial.println("info: " + String(health.status) +", " + String(health.error_code));
  // detected...
  lidar.startScan();

  esp_task_wdt_deinit();
  // Asign lidar Task to core 0
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

  // start lidar's motor rotating at max allowed speed
  analogWrite(pinLIDAR_motor, 255);
  delay(500);

  // wait until y coordinate is calculated
  while (readDistance(0) == 0)
  {
    delay(100);
  }
  setYcoord(readDistance(0));
  delay(500);

  #if PRACTICE_MODE == false
  digitalWrite(pinLED_verde, HIGH);
  while (digitalRead(pinBoton)) {
    while (commSerial.available())
    {
      receiveData();
    }
  }
  digitalWrite(pinLED_verde, LOW);
  #endif
  delay(1000);

  // start driving (set a speed to the car and initialize the mpu)
  setSpeed(4);
  mimpu.measureFirstMillis();
}

void loop() {
  // put your main code here, to run repeatedly:

  // receive data from the intercommunication serial
  while (commSerial.available())
  {
    receiveData();
  }

  // update mpu's angle
  mimpu.UpdateAngle();

  // repeat position functions every 32ms
  static uint32_t prev_ms_position = millis();
  if (millis() > prev_ms_position) {
    if (encoderMeasurement != prev_encoderMeasurement) {
      // calculate the increment in position and add it
      double dy = (encoderMeasurement - prev_encoderMeasurement) * cos(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      double dx = (encoderMeasurement - prev_encoderMeasurement) * sin(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      prev_encoderMeasurement = encoderMeasurement;
      xPosition -= dx; // x -> + derecha - izquierda
      yPosition += dy;
      iteratePositionPID();
    }
    prev_ms_position = millis() + 32;
  }

  // send telemetry every 100ms
  static uint32_t prev_ms_tele = millis();
  if (millis() > prev_ms_tele+500)
  {

    /*FORMATO TELEMETRIA
    |inicioTX            |TipoPaquete|Datos|
      0xAA,0xAA,0xAA,0xAA,0x--,0x--...0x--
    */
   /*ENVIAMOS PAQUETE TIPO 4 DISTANCIAS*/

   /***************************************************************** 
    for(int i = 0; i<4; i++){
      teleSerial.write(0xAA);
    }
    teleSerial.write(04);
    uint16_t zi=0;
    while (zi < 360)
    {
        teleSerial.write(distances[zi]>>8);
        teleSerial.write(distances[zi]&0x00ff);
        zi++;
    }*/
   
    /*/ENVIAMOS PAQUETE TIPO 3 CALIDAD MEDIDA/
    for(int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(03);
    uint16_t pi=0;
    while (pi < 360){
        teleSerial.write(distancesArray[1][pi]);
        pi++;
    }*/

    /*ENVIAMOS PAQUETE TIPO 5 INFORMACION GENERAL*/
                /*
            --Posicion x 8 bytes
            --Posición y 8 bytes
            --Posición x Objetivo 8 bytes
            --Posición y Objetivo 8 bytes
            --Encoder 32 uint32
            --Estado 8bits  uint
            --batería 8bits uint
            --Ángulo 16 float
            --Angulo Objetivo 16 float
            --Cámara firma1 Detectada 1 byte
            --Cámara firma1 x 8 bits
            --Cámara firma1 y 8 bits
            --Cámara firma2 Detectada 1byte
            --Cámara firma2 x 8bits
            --Cámara firma2 y 8bits
            
            |XXXX|YYYY|MMMM|NNNN|QQQQ|W|E|RRRR|TTTT|U|I|O|A|S|D
             0000 0000 0111 1111 1112 2 2 2222 2223 3 3 3 3 3 3
             1234 5678 9012 3456 7890 1 2 3456 7890 1 2 3 4 5 6
            */
    for(int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(05);
    long posXLong = long(xPosition);
    long posYLong = long(yPosition);
    long posXObjLong = long(objectivePosition);
    long posYObjLong = (turnSense==-1)?1:(turnSense==1)?2:0;
    long anguloLong = long(mimpu.GetAngle());
    long anguloObjLong = long(objectiveDirection);
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
    for (byte i=0; i<10; i++) teleSerial.write(byte(00));
    
    prev_ms_tele = millis();
  }


  // check turn every 50ms
  static uint32_t prev_ms_turn = millis();
  if (millis() > prev_ms_turn) {
    checkTurn();
    prev_ms_turn = millis() + 50;
  }

  // repeat direction pid iterations every 20ms
  static uint32_t prev_ms_direction = millis();
  if (millis() > prev_ms_direction) {
    actual_directionError = constrain(directionError(mimpu.GetAngle(), objectiveDirection), -127, 127);
    int _setAngle = servoKP * actual_directionError + servoKD * (actual_directionError - prev_directionError);
    if(_setAngle != prev_setAngle) {
      setSteering(_setAngle);
      prev_setAngle = _setAngle;
    }
    prev_directionError = actual_directionError;
    prev_ms_direction = millis() + 20;
  }

  // state machine
  switch (estado)
  {
  case e::Inicio:
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
          setSpeed(5);
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
  uint8_t _speed = (abs(speed) << 1) | ((speed >= 0) ? 0 : 1);
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

      if (distance > 100 && distance < 3000)
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
  objectiveDirection = constrain(positionKP * positionError + positionKD * (positionError - prev_positionError), -90, 90);
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
  yPosition = 3000 - f - 150;
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
  while(posicion >= 0 ){
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
/*
struct dataPacket
{
  byte packetType;
  byte size;
};

*/