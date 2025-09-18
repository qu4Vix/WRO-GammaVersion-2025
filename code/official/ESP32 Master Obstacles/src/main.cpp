#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
//#include "credentials.h"
#include "pinAssignments.h"

#include <rom/rtc.h>
#include <esp_task_wdt.h>

// Practice mode (Button desabled) for easy launches while testing
#define PRACTICE_MODE false

// Enables wifi functions when true
#define ENABLE_WIFI false
#define ENABLE_TELEMETRY true

/*#if ENABLE_WIFI == true
#include <OTAUpdate.h>
#include <Telemetry.h>

IPAddress receiversIP(192, 168, 0, 102);
uint16_t receiversPort = 4210;
uint16_t udpPort = 1234;
uint16_t otaPort = 80;

Updater miota(otaPort);
TelemetryManager telemetry(receiversIP, receiversPort);
#endif
*/
// Speeds
#define StartSpeed 2
#define CruisiereSpeed 5
#define NormalSpeed 3

/*
 *  Speed infromation
 *  Minimum speed: 2
 *  Safe speed for recognition: 3
 *  Safe speeds for cruisiere: 5,6
 *  Risky speed: 7
 *
 * 
 */

// Servo and direction variables

#define servoKP 2.5
#define servoKD 0
int prev_setAngle;
int actual_directionError;
int prev_directionError;
float objectiveDirection;

// battery level variable
uint8_t bateria;

// camera signatures

#define GreenSignature 1
#define RedSignature 2

bool firma1Detectada = false;
uint8_t firma1X = 18;
uint8_t firma1Y = 19;
bool firma2Detectada = false;
uint8_t firma2X = 20;
uint8_t firma2Y = 21;

// conversion between mm and encoder counts
#define MMperEncoder 1.41

// List of possible states for the car
enum e {
  Inicio,
  Recto,
  Final,
  Arrancar,
  Reposicionar,
  Reinicio, // Not necessary -------------------------------------------------------------------------------------------------
  Posicionamiento,
  EntradaFase1,
  EntradaFase2,
  EntradaFase3,
  EntradaFase4,
  Esperar,
  Prueba
};
uint8_t estado = e::Arrancar;

// track constants

// size of the map (mm)
#define mapSize 3000
// width of the lanes
#define trackWidth 1000
// coordinate of the central lane (mm)
#define trackCenter 500
// coordinate of the lateral lane (mm)
#define trackLateral 300
// distance to path on which the car should try to turn (mm)
#define turnOffset 300

// Car constants
#define carWidth 150
#define carLength 240
#define lidarToImu 150

// journey variables

// number of turns
uint8_t giros = 0;
uint8_t totalGiros = 0;
// section in which the car is
uint8_t tramo = 1;
// sense of turn
int8_t turnSense = 0;
// whether you have to turn clockwise or not
bool turnClockWise;
// whether we are moving forwards (+1) or backwards (-1), in order to account for corrections in the PID. We initialize it at 0 (not moving)
int8_t motionDirection = 0;
bool isRecognizing = true;
bool isParking = false;

// encoder variables

int32_t encoderMeasurement;
int32_t prev_encoderMeasurement;

// lidar measurement variables

uint16_t distances[360];
static uint16_t distancesMillis[360];
uint16_t distancia0;
uint16_t distancia90;
uint16_t distancia270;

// car position on the map (origin at the bottom-left corner)

// x position of the car (increases to the right)
double xPosition = 0;
// y position of the car (increases upwards)
double yPosition = 0;

#define posYmagica 1500 // value of initialization of the variable yPosition so that program starts normally, even though the yPos is properly obtained in state e::Volver

double startXposition;
double marcaPos;
unsigned long marcaMillis;
byte marcaEstado;

// position PID controller variables

#define positionKP 0.25  // different from phase 1 for some reason --------------------------------------------------------------------
#define positionKD 1
#define positonKPmagico 7.5
#define positionKPaparcar 3.5
#define positionKDaparcar 12.5 //Ajustar para que haga las maniobras bien, y todo el lio...
float KPActual = positonKPmagico;
float KDActual = positionKD;
int objectivePosition = 0;
float positionError;
float prev_positionError;
bool fixXposition = true;
bool fixInverted = true;

// trajectory management variables

uint16_t tramos[2][8] = {
  {500,500,2500,2500,2500,2500,500,500},
  {mapSize - trackCenter, mapSize - trackCenter, mapSize - trackCenter, mapSize - trackCenter, trackCenter, trackCenter, trackCenter, trackCenter}
};
uint8_t arrayBloques[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint16_t blockPaths[2][4][3] =
{
  { // anticlockwise
    {2500, mapSize - trackCenter - trackLateral, mapSize - 380}, //Center, left, right
    {2500, mapSize - trackCenter - trackLateral, mapSize - trackCenter + trackLateral},
    {500, trackCenter + trackLateral, trackCenter - trackLateral},
    {500, trackCenter + trackLateral, trackCenter - trackLateral}
  },
  { // clockwise
    {500, 380, trackCenter + trackLateral},
    {2500, mapSize - trackCenter + trackLateral, mapSize - trackCenter - trackLateral}, // Center, left, right
    {2500, mapSize - trackCenter + trackLateral, mapSize - trackCenter - trackLateral},
    {500, trackCenter - trackLateral, trackCenter + trackLateral}
  }
};
const uint16_t blockPositions[2][4][3] = 
{
  { // Anticlockwise
    {950, 1400, 1900}, // First, second, third block
    {2100, 1600, 1050},
    {2100, 1600, 1050},
    {950, 1400, 1900}
  },
  { // Clockwise
    {950, 1400, 1900},
    {950, 1400, 1900}, // First, second, third block
    {2100, 1600, 1050},
    {2100, 1600, 1050}
  }
};
uint8_t lastBlock;
bool senseDirectionChanged = false;
bool bloqueEnMedio = false;
uint16_t parkingY;

// object declarations

MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;
HardwareSerial commSerial(1);
TaskHandle_t Task1;
HardwareSerial teleSerial(0);

// calculate the error in the direction
int directionError(int bearing, int target);
float directionError(double bearing, int target);

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

// functions for the management of the car's position

void iteratePositionPID();  // invoke an iteration of the pid controller for the position
void turn();            // turn
void setXcoord(uint16_t i);   // set the coordinate x axis
void setYcoord(uint16_t f);   // set the coordinate y axis
void decideTurn();  // detect the sense of turn
void checkTurn();   // check wether you have to turn or not

void changeLane(uint8_t _tramo);
bool setCoordTramo(uint8_t tramo, uint16_t leftCoord, uint16_t rightCoord);
void correctLane(uint8_t _tramo);
void changeDrivingDirection();

void moveCamera(int8_t angle);
void autoMoveCamera();
void enableCamera();

void saveParkingPosition();

void pitiditos(int num){
  while(num > 0){
    digitalWrite(pinBuzzer, HIGH);
    delay(50);
    digitalWrite(pinBuzzer,LOW);
    delay(50);
    num--;
  }
}

void estadoEsperar(byte estadoObjetivo, uint16_t delay) {
  pitiditos(1);
  marcaMillis = millis() + delay;
  marcaEstado = estadoObjetivo;
  estado = e::Esperar;
}

void setup() {
  // put your setup code here, to run once:
  setPinModes();
  pitiditos(1);
  #if ENABLE_TELEMETRY == true
  // begin telemetry serial
  teleSerial.begin(1000000, SERIAL_8N1, telemetriaRX, telemetriaTX);
  #else
  // begin serial
  Serial.begin(115200);
  #endif
  // begin esp32 intercommunication serial
  commSerial.begin(1000000, SERIAL_8N1, pinRX, pinTX);

  // set all the pin modes
  
  mimpu.SetDebugLedPin(pinLED_rojo);
/*
  #if ENABLE_WIFI == true
    miota.WiFiInit();
    miota.SetStaticIP(250);
    miota.OTAInit();

    telemetry.StartUDP(udpPort);
  #endif
*/
  // configure the mpu
  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();
  pitiditos(2);
  // begin the lidar
  lidar.begin(lidarSerial);
  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  //Serial.println("info: " + String(health.status) +", " + String(health.error_code));
  // detected...
  lidar.startScan();
  pitiditos(3);
  //esp_task_wdt_deinit();
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
  pitiditos(4);
  // start lidar's motor rotating at max allowed speed
  analogWrite(pinLIDAR_motor, 255);
  delay(500);

  // initialize y position and wait until the turn sense is decided and the x coordinate is calculated
  digitalWrite(pinLED_rojo, HIGH);
  yPosition = posYmagica;
  while (turnSense==0)
  {
    decideTurn();
  }
  startXposition = xPosition;
  digitalWrite(pinLED_rojo, LOW);
  
  // handle this in case the car has to turn around in final lap --------------------------------------------------------------
  //if (yPosition >= 1500) bloqueEnMedio = true;
  
  #if PRACTICE_MODE == true
  // Receive data from the intercommunication serial
  while (commSerial.available())
  {
    receiveData();
  }
  #else
  // Waits until the start button is pressed while reading from the intercommunication serial
  digitalWrite(pinLED_amarillo, HIGH);
  while (digitalRead(pinBoton)) {
    while (commSerial.available())
    {
      receiveData();
    }
  }
  digitalWrite(pinLED_amarillo, LOW);
  #endif
  prev_encoderMeasurement = encoderMeasurement;
  delay(500);

  // move camera away from parking lot
  autoMoveCamera();
  firma1Detectada=firma1Detectada=0;
  // set default first lane path to the inner one in order for the camera to look away from the block
  if (turnClockWise) arrayBloques[0] = arrayBloques[1] = GreenSignature;
  else arrayBloques[0] = arrayBloques[1] = RedSignature;
  // set the objective position of the unparking
  objectivePosition = 2500-2000*turnClockWise;
  // start driving (set a speed to the car and initialize the mpu)
  setSpeed(StartSpeed);
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
  if (millis() >= prev_ms_position) {
    if (encoderMeasurement != prev_encoderMeasurement) {
      // calculate the increment in position and add it
      double dy = (encoderMeasurement - prev_encoderMeasurement) * cos(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      double dx = (encoderMeasurement - prev_encoderMeasurement) * sin(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      prev_encoderMeasurement = encoderMeasurement;
      xPosition -= dx; // x -> + derecha - izquierda
      yPosition += dy;
    }
    iteratePositionPID();
    prev_ms_position = millis() + 32;
  }

  static uint32_t prev_Cam = millis();
  if (millis() >= prev_Cam) {
    if (isRecognizing) {
      autoMoveCamera();
    }
    prev_Cam = millis() + 100;
  }

  #if ENABLE_TELEMETRY == true
  // send telemetry every 100ms
  static uint32_t prev_ms_tele = millis();
  if (millis() > prev_ms_tele + 100)
  {
    /*FORMATO TELEMETRIA
    |inicioTX            |TipoPaquete|Datos|
      0xAA,0xAA,0xAA,0xAA,0x--,0x--...0x--
    */
   /*ENVIAMOS PAQUETE TIPO 4 DISTANCIAS*/
   /*
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
    
    for(int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
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
    enviarDato((byte*)&distancia0,sizeof(distancia0));
    enviarDato((byte*)&distancia90,sizeof(distancia90));
    enviarDato((byte*)&distancia270,sizeof(distancia270));
    
    for(int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(byte(07));
    enviarDato((byte*)&firma1Detectada,sizeof(firma1Detectada));
    enviarDato((byte*)&firma1X,sizeof(firma1X));
    enviarDato((byte*)&firma1Y,sizeof(firma1Y));
    teleSerial.write(0x00);teleSerial.write(0x00);
    enviarDato((byte*)&firma2Detectada,sizeof(firma2Detectada));
    enviarDato((byte*)&firma2X,sizeof(firma2X));
    enviarDato((byte*)&firma2Y,sizeof(firma2Y));
    teleSerial.write(0x00);teleSerial.write(0x00);
    
    for(int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(byte(01));
    enviarDato((byte*)&arrayBloques,sizeof(arrayBloques));
    teleSerial.write(0x00);teleSerial.write(0x00);
    
    prev_ms_tele = millis();
  }
  #endif

  // check turn every 50ms
  static uint32_t prev_ms_turn = millis();
  if (millis() > prev_ms_turn) {
    // No need to turn around in the final lap this year, so we will comment the function
    //if (totalGiros == 8) changeDrivingDirection();

    // Check whether we have to turn or steer to avoid color blocks
    if (!isParking) checkTurn();
    prev_ms_turn = millis() + 50;
  }

  // repeat direction pid iterations every 20ms
  static uint32_t prev_ms_direction = millis();
  if (millis() > prev_ms_direction) {
    actual_directionError = constrain(directionError(mimpu.GetAngle(), objectiveDirection), -127, 127);
    int _setAngle = servoKP * actual_directionError + servoKD * (actual_directionError - prev_directionError);
    if(_setAngle != prev_setAngle) {
      setSteering(_setAngle * motionDirection);
      prev_setAngle = _setAngle;
    }
    prev_directionError = actual_directionError;
    prev_ms_direction = millis() + 20;
  }

  // state machine
  switch (estado)
  {
  case e::Inicio:
    //firma1Detectada = firma2Detectada = false; choose what to do with this line too --------------------------------------------------------
    //objectivePosition = xPosition; //decide what to do with this line ----------------------------------------------------------------------
    vTaskDelete(Task1);
    analogWrite(pinLIDAR_motor, 0);
    estado = e::Recto;
    setSpeed(NormalSpeed);
  break;
  case e::Recto:
    if (totalGiros == 12) {
      estado = e::Posicionamiento;
    }
    if (totalGiros == 5) {
      setSpeed(CruisiereSpeed);
    }
  break;
  case e::Final:  // we probably want to be redirected to here after we finish the parking manouver ---------------------------------------------
    if (yPosition >= 1200) {
      setSpeed(0);
    }
  break;

  case e::Arrancar:
    //Ajustar el avance para que no se pase del bloque y tal...
    if (xPosition*turnSense <= startXposition*turnSense - 150)
    {
      setSpeed(0);
      delay(10);
      estadoEsperar(e::Reposicionar, 3000);
    }
  break;
  case e::Reposicionar:
    distancia0 = readDistance(0);
    if (distancia0)
    {
      setYcoord(distancia0);
      saveParkingPosition();
      KPActual = positionKP;
      estado = e::Inicio;
      //estadoEsperar(e::Inicio, 15000);
    }
  break;

  case e::Posicionamiento:
    objectivePosition = startXposition - 225*turnSense;
    //KPActual = positionKP;
    isParking = true;
    setSpeed(StartSpeed);
    estado = e::EntradaFase1;
  break;
  case e::EntradaFase1:
    if (yPosition >= parkingY)
    {
      setSpeed(0);
      estadoEsperar(e::EntradaFase2, 500);
    }
  break;
  case e::EntradaFase2:
    KPActual = positionKPaparcar;
    KDActual = positionKDaparcar;
    setSpeed(-StartSpeed);
    objectivePosition = startXposition;
    estado = e::EntradaFase3;
  break;
  case e::EntradaFase3:
    if (yPosition <= 1650-500*turnClockWise)
    {
      KPActual = positionKP;
      KDActual = positionKD;
      setSpeed(0);
      /*
      delay(50);
      marcaPos = yPosition;
      setSpeed(StartSpeed); // probably not necessary if we stop on point -----------------------------------------------------------
      estado = e::EntradaFase4;*/
    }
  break;
  case e::EntradaFase4:
    if (yPosition >= marcaPos+50)
    {
      setSpeed(0);
    }
  break;
  /*
  maniobra alternativa
  parar a 1900
  marcha atras y girar tuedas a tope hasta que marque 90º (*turnSense)
  girar ruedas hacia el otro lado y volver hacia atras hasta que el angulo sea 0
  parar
  */

  case e::Esperar:
    if (millis() >= marcaMillis) {
      estado = marcaEstado;
    }
  break;
  }
}

int directionError(int bearing, int target) {
  int error = target - bearing;
  return error;
}

float directionError(double bearing, int target) {
  float error = target - bearing;
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
  /*
  // reduce speed when doing sharp turns
  if ((giros >= 5) & (giros < 12)) {
    if (abs(angle) >= 60) {
      setSpeed(NormalSpeed);
    } else setSpeed(CruisiereSpeed);
  }*/
  angle = constrain(angle, -90, 90);
  uint8_t _angle = angle + 90;
  commSerial.write(2);
  commSerial.write(_angle);
}

void receiveData() {
  uint8_t firstByte;
  commSerial.readBytes(&firstByte, 1);
  if (firstByte == 7) { // Encoder data
    uint8_t bytes[4];
    commSerial.readBytes(bytes, 4);
    encoderMeasurement = 0;
    for (uint8_t iteration; iteration < 4; iteration++) {
      encoderMeasurement = encoderMeasurement | bytes[iteration] << (8*iteration);
    }
  } else
  if (firstByte == 6) { // Batery voltage
    uint8_t tensionValue;
    commSerial.readBytes(&tensionValue, 1);
    bateria = tensionValue;
    manageTension(tensionValue);
  } else 
  if (firstByte == 5) { // Camera data
    uint8_t Signature;
    uint8_t SignatureX;
    uint8_t SignatureY;
    commSerial.readBytes(&Signature, 1);
    commSerial.readBytes(&SignatureX, 1);
    commSerial.readBytes(&SignatureY, 1);
    // solo aceptar firmas en la primera vuelta
    if (isRecognizing) {
      //pitiditos(1);
      firma1Detectada = (Signature==1);
      firma2Detectada = (Signature==2);
      firma1X = SignatureX;
      firma1Y = SignatureY;
    }
  }
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
  
  // copy the distances array to avoid writing what is being read
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

      float angulo = angle - mimpu.GetAngle();
      if (angulo < 0)
      {
        angulo = 360 + angulo;
      }
      else if (angulo >= 360)
      {
        angulo -= 360;
      }
      uint16_t index = getIndex(angulo);
      
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
  enableCamera();
  switch ((tramo+1) * turnSense)
  {
  case -2:
    objectivePosition = blockPaths[turnClockWise][1][arrayBloques[2]];
    fixInverted = false;
    fixXposition = false;
    tramo++;
    break;
  
  case -4:
    objectivePosition = blockPaths[turnClockWise][2][arrayBloques[4]];
    fixInverted = false;
    fixXposition = true;
    tramo++;
    break;

  case -6:
    objectivePosition = blockPaths[turnClockWise][3][arrayBloques[6]];
    fixInverted = true;
    fixXposition = false;
    tramo++;
    break;

  case -8:
    objectivePosition = blockPaths[turnClockWise][0][arrayBloques[0]];
    fixInverted = true;
    fixXposition = true;
    tramo = 0;
    break;
  
  case 2:
    objectivePosition = blockPaths[turnClockWise][1][arrayBloques[2]];
    fixInverted = true;
    fixXposition = false;
    tramo++;
    break;
  
  case 4:
    objectivePosition = blockPaths[turnClockWise][2][arrayBloques[4]];
    fixInverted = false;
    fixXposition = true;
    tramo++;
    break;

  case 6:
    objectivePosition = blockPaths[turnClockWise][3][arrayBloques[6]];
    fixInverted = false;
    fixXposition = false;
    tramo++;
    break;

  case 8:
    objectivePosition = blockPaths[turnClockWise][0][arrayBloques[0]];
    fixInverted = true;
    fixXposition = true;
    tramo = 0;
    break;
  }
  giros++;
  totalGiros++;
}

void setXcoord(uint16_t i) {
  xPosition = i;
}

void setYcoord(uint16_t f) {
  yPosition = mapSize - lidarToImu * cos(mimpu.GetAngle() * (M_PI/180)) - f;
}

void checkTurn() {
  switch ((tramo+1) * turnSense)
  {
  case 0:
    if (setCoordTramo(1, 2200, 2800)) {
      objectivePosition = blockPaths[0][0][arrayBloques[1]] - 2500;
    }
    break;
  case -1:
    if (setCoordTramo(0, 200, 800)) correctLane(0);
    if (yPosition >= 1000) changeLane(1);
    break;
  
  case -2:
    if ((yPosition <= 1600) && setCoordTramo(1, 200, 800)) correctLane(1);
    if (yPosition >= 2000) turn();
    if (totalGiros >= 4) {
      isRecognizing = false;
      moveCamera(0);
    }
    break;

  case -3:
    if (setCoordTramo(2, 2800, 2200)) correctLane(2);
    if (xPosition >= 1000) changeLane(3);
    break;

  case -4:
    if ((xPosition <= 1600) && (setCoordTramo(3, 2800, 2200))) correctLane(3);
    if (xPosition >= 2000) turn();
    break;

  case -5:
    if (setCoordTramo(4, 2800, 2200)) correctLane(4);
    if (yPosition <= 2000) changeLane(5);
    break;

  case -6:
    if ((yPosition >= 1400) && (setCoordTramo(5, 2800, 2200))) correctLane(5);
    if (yPosition <= 1000) turn();
    break;

  case -7:
    if (setCoordTramo(6, 200, 800)) correctLane(6);
    if (xPosition <= 2000) changeLane(7);
    break;

  case -8:
    if ((xPosition >= 1400) && (setCoordTramo(7, 200, 800))) correctLane(7);
    if (xPosition <= 1000) turn();
    break;

  case 1:
    if (setCoordTramo(0, 2200, 2800)) correctLane(0);
    if (yPosition >= 1000) changeLane(1);
    break;
  
  case 2:
    if ((yPosition <= 1600) && setCoordTramo(1, 2200, 2800)) correctLane(1);
    if (yPosition >= 2000) turn();
    if (totalGiros >= 4) {
      isRecognizing = false;
      moveCamera(0);
    }
    break;

  case 3:
    if (setCoordTramo(2, 2200, 2800)) correctLane(2);
    if (xPosition <= 2000) changeLane(3);
    break;

  case 4:
    if ((xPosition >= 1400) && (setCoordTramo(3, 2200, 2800))) correctLane(3);
    if (xPosition <= 1000) turn();
    break;
  
  case 5:
    if (setCoordTramo(4, 800, 200)) correctLane(4);
    if (yPosition <= 2000) changeLane(5);
    break;
  
  case 6:
    if ((yPosition >= 1400) && (setCoordTramo(5, 800, 200))) correctLane(5);
    if (yPosition <= 1000) turn();
    break;

  case 7:
    if (setCoordTramo(6, 800, 200)) correctLane(6);
    if (xPosition >= 1000) changeLane(7);
    break;

  case 8:
    if ((xPosition <= 1600) && (setCoordTramo(7, 800, 200))) correctLane(7);
    if (xPosition >= 2000) turn();
    break;
  }
}

void decideTurn(){
  distancia90 = readDistance(90);
  distancia270 = readDistance(270);
  if (distancia90 > distancia270 && distancia90 > 500)
  {
    turnSense = -1;
    turnClockWise = true;
    setXcoord(1000-distancia90);
  }
  else if (distancia270 > distancia90 && distancia270 > 500)
  {
    turnSense = 1;
    turnClockWise = false;
    setXcoord(distancia270+2000);
  }
  else turnSense = 0;
}

void changeLane(uint8_t _tramo) {
  objectivePosition = blockPaths[turnClockWise][uint8_t(_tramo/2)][arrayBloques[_tramo]];
  enableCamera();
  tramo++;
}

void correctLane(uint8_t _tramo) {
  objectivePosition = blockPaths[turnClockWise][uint8_t(_tramo/2)][arrayBloques[_tramo]];
}

void enviarDato(byte* pointer, int8_t size){
  int8_t posicion = size - 1;   //recorreremos la memoria desde el de mas valor hara el de menos, ya que el 
                          //receptor espera ese orde MSB
  while (posicion >= 0 ) {
    teleSerial.write(pointer[posicion]);
    posicion--;
  }
}

bool setCoordTramo(uint8_t _tramo, uint16_t leftCoord, uint16_t rightCoord) {
  if (firma1Detectada) {
    arrayBloques[_tramo] = GreenSignature;
    lastBlock = GreenSignature;
    //tramos[turnClockWise][_tramo] = leftCoord; // ------------------------ innecesario este año -------------------------------- 
    firma1Detectada = false;
    return true;
  }
  if (firma2Detectada) {
    arrayBloques[_tramo] = RedSignature;
    lastBlock = RedSignature;
    //tramos[turnClockWise][_tramo] = rightCoord; // ----------------------- innecesario este año --------------------------------
    firma2Detectada = false;
    return true;
  }
  return false;
}

void changeDrivingDirection() {
  if (senseDirectionChanged == 0) {
    if (yPosition >= 1500 - 500*bloqueEnMedio) {
      if (lastBlock == RedSignature) {
        xPosition = 3000 - xPosition;
        yPosition = 3000 - yPosition;
        turnSense *= -1;
        turnClockWise = !turnClockWise;
        tramo = 1;
        mimpu.addAngle(900*turnSense);
        giros = 0;
        uint8_t colorBlocks[8];
        for (uint8_t i = 0; i<8; i++) {
          colorBlocks[i] = arrayBloques[i];
        }
        for (uint8_t i = 0; i<8; i++) {
          uint8_t index = 8-1 - i + 2;
          if (index > 7) index -= 8;
          arrayBloques[i] = colorBlocks[index];
        }
        correctLane(1);
        setSpeed(NormalSpeed);
      }
      senseDirectionChanged = true;
    }
  }
}

void moveCamera(int8_t angle) {
  uint8_t _angle = constrain(angle, -90, 90) + 90;
  commSerial.write(3);
  commSerial.write(_angle);
}

// angle in degrees, between 0 and 360º, with respect to the Y axis, anticlockwise
int atan3(double _dx, double dy) {
  double dx = _dx * turnSense;
  int beta = 180 / M_PI * atan(-dx / dy);
  if (dy > 0 && dx < 0) return beta;
  else if (dy > 0 && dx > 0) return beta + 360;
  else if (dy < 0) return beta + 180; // here we get angles between 90 and 270
  else if (dy == 0 && dx > 0) return -90; // angle decreases in the clockwise (positive x) direction
  else if (dy == 0 && dx < 0) return 90; // angle increases in the anti-clockwise (negative x) direction
  else return 0; // indeterminate
}

void calculateCameraAngle(uint16_t bX, uint16_t bY) {
  int _ang = atan3(bX - xPosition, bY - yPosition) * turnSense; // this angle is given with respect to the Y axis, increasing anti-cloclwise -like the IMU angle
  moveCamera(_ang - mimpu.GetAngle()); // check if the camera 90º (Servo.write(0)) is the same direction as the IMU 90º
}

void autoMoveCamera() {
  switch ((tramo+1) * turnSense)
  {
  case -2:
    if (yPosition <= 1450)
      calculateCameraAngle(600, 1500);  // blocks on the first lane can only be on the outer position (x=600 when clockwise)
    else if (yPosition <= 1950)
      calculateCameraAngle(600, 2000);  // blocks on the first lane can only be on the outer position (x=600 when clockwise)
    else {
      calculateCameraAngle(1000, 2500);
      isRecognizing = false;
    }
  break;

  case -3:
    if (xPosition <= 950)
      calculateCameraAngle(1000, 2500);
    else {
      calculateCameraAngle(1500, 2500);
      isRecognizing = false;
    }
  break;

  case -4:
    if (xPosition <= 1450)
      calculateCameraAngle(1500, 2500);
    else if (xPosition <= 1950)
      calculateCameraAngle(2000, 2500);
    else {
      calculateCameraAngle(2500, 2000);
      isRecognizing = false;
    }
  break;

  case -5:
    if (yPosition >= 2050)
      calculateCameraAngle(2500, 2000);
    else {
      calculateCameraAngle(2500, 1500);
      isRecognizing = false;
    }
  break;

  case -6:
    if (yPosition >= 1650)
      calculateCameraAngle(2500, 1500);
    else if (yPosition >= 1050)
      calculateCameraAngle(2500, 1000);
    else {
      calculateCameraAngle(2000, 500);
      isRecognizing = false;
    }
  break;

  case -7:
    if (xPosition >= 2050)
      calculateCameraAngle(2000, 500);
    else {
      calculateCameraAngle(1500, 500);
      isRecognizing = false;
    }
  break;

  case -8:
    if (xPosition >= 1650)
      calculateCameraAngle(1500, 500);
    else if (xPosition >= 1050)
      calculateCameraAngle(1000, 500);
    else {
      calculateCameraAngle(600, 1000);
      isRecognizing = false;
    }
  break;

  case -1:
    if (yPosition <= 950)
      calculateCameraAngle(600, 1000);  // blocks on the first lane can only be on the outer position (x=600 when clockwise)
    else {
      calculateCameraAngle(600, 1500);
      isRecognizing = false;
    }
  break;

  case 2:
    if (yPosition <= 1450)
      calculateCameraAngle(2380, 1500); // blocks on the first lane can only be on the outer position (x=2380 when anti-clockwise)
    else if (yPosition <= 1900)
      calculateCameraAngle(2380, 2000); // blocks on the first lane can only be on the outer position (x=2380 when anti-clockwise)
    else {
      calculateCameraAngle(2000, 2500);
      isRecognizing = false;
    }
  break;

  case 3:
    if (xPosition >= 2050)
      calculateCameraAngle(2000, 2500);
    else {
      calculateCameraAngle(1500, 2500);
      isRecognizing = false;
    }
  break;

  case 4:
    if (xPosition >= 1650)
      calculateCameraAngle(1500, 2500);
    else if (xPosition >= 1100)
      calculateCameraAngle(1000, 2500);
    else {
      calculateCameraAngle(500, 2000);
      isRecognizing = false;
    }
  break;

  case 5:
    if (yPosition >= 2050)
      calculateCameraAngle(500, 2000);
    else {
      calculateCameraAngle(500, 1500);
      isRecognizing = false;
    }
  break;

  case 6:
    if (yPosition >= 1650)
      calculateCameraAngle(500, 1500);
    else if (yPosition >= 1100)
      calculateCameraAngle(500, 1000);
    else {
      calculateCameraAngle(1000, 500);
      isRecognizing = false;
    }
  break;

  case 7:
    if (xPosition <= 950)
      calculateCameraAngle(1000, 500);
    else {
      calculateCameraAngle(1500, 500);
      isRecognizing = false;
    }
  break;

  case 8:
    if (xPosition <= 1450)
      calculateCameraAngle(1500, 500);
    else if (xPosition <= 1900)
      calculateCameraAngle(2000, 500);
    else {
      calculateCameraAngle(2380, 1000);
      isRecognizing = false;
    }
  break;

  case 1:
    if (yPosition <= 950)
      calculateCameraAngle(2380, 1000); // blocks on the first lane can only be on the outer position (x=2380 when anti-clockwise)
    else {
      calculateCameraAngle(2380, 1500);
      isRecognizing = false;
    }
  break;
  }
}

// enables camera when the total number of turns is less than 5 and sets the camera signatures to false
void enableCamera() {
  if (totalGiros < 5) {
    firma1Detectada = firma2Detectada = false;
    isRecognizing = true;
  }
}

void saveParkingPosition() {
  if (distancia0 > 1500) {
    parkingY = 1375;
  } else {
    parkingY = 1935;
  }
}