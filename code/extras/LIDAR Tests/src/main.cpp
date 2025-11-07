#include <Arduino.h>
#include <RPLidar.h>

#include "ransacAlgorithm.h"
#include "lidarHandling.h"

#define PIN_LIDAR_MOTOR 5 // The PWM pin for control the speed of RPLIDAR's motor.
#define PIN_Buzzer 32

HardwareSerial lidarSerial(2);
RPLidar lidar;

TaskHandle_t Task1;

polarPoint2D lidarBuffer[365];
uint16_t measurementsIndex = 0;
lidarStorage * pStorage;

bool reading = false;
bool writing = false;

uint16_t blockPositions[2][12][2] = {
  // Clockwise
  {
    {}
  }
};

void Task1Code(void * pvParameters);
float readDistance(uint16_t angle);

// put your setup code here, to run once:
void setup() {
  pinMode(PIN_LIDAR_MOTOR, OUTPUT);
  pinMode(PIN_Buzzer, OUTPUT);

  Serial.begin(115200);
  lidar.begin(lidarSerial);

  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  // detected...
  lidar.startScan();
  analogWrite(PIN_LIDAR_MOTOR, 255);
  delay(500);

  // Asign task 1 to core 0
  xTaskCreatePinnedToCore(
    Task1Code,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    1);
}

// put your main code here, to run repeatedly:
void loop() {
  // Send data every 25000ms
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms) {
    unsigned long ms = millis();
    float distance = readDistance(0);
    Serial.println("Distance: " + String(distance) + ",    Millis: " + String(millis() - ms));
    prev_ms = millis() + 5000;
  }
}

// Create code for task1
void Task1Code(void * pvParameters) {
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());

  for (;;) {
    if (reading);
    else {
      if (IS_OK(lidar.waitPoint())) {
        double distance = lidar.getCurrentPoint().distance; //distance value in mm unit
        double angle    = lidar.getCurrentPoint().angle; //anglue value in degree
        bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
        if (startBit) {
          writing = true;
          pStorage->~lidarStorage();
          pStorage = new lidarStorage(measurementsIndex);
          pStorage->setArray(lidarBuffer, measurementsIndex);
          measurementsIndex = 0;
          writing = false;
        }
        // store the new measurement in the buffer (range and bearing, the angle between 180 and -180 for easy substractions)
        lidarBuffer[measurementsIndex] = polarPoint2D{distance, (angle > 180)?(angle - 360):angle};
        measurementsIndex++;
      }
    }
  }
}

// Angle from 0 to 359
float readDistance(uint16_t angle) {
  while (writing);
  Line2D lines[MAX_LANDMARKS];
  reading = true;
  int output = exeRANSAC(pStorage, lines, 0);
  reading = false;
  if (output == -1) {
    digitalWrite(PIN_Buzzer, HIGH);
    return 0;
  }
  /*polarPoint2D polarLines[output];
  for (uint8_t l = 0; l < output; l++) {
    Point2D perp = PerpPointOnLine(lines[l], Point2D{0,0});
    polarLines[l];
  }*/
  for (uint8_t l = 0; l < output; l++) {
    if (abs(180/PI*atan(lines[l].a) - angle) < 5) {
      return DistanceToLine(lines[l], Point2D{0,0});
    }
  }
  return 0;
}