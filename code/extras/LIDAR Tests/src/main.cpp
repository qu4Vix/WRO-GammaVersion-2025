#include <Arduino.h>
#include <RPLidar.h>

#define PIN_LIDAR_MOTOR 5 // The PWM pin for control the speed of RPLIDAR's motor.

HardwareSerial lidarSerial(2);
RPLidar lidar;

TaskHandle_t Task1;

struct lidarMeasurement
{
  uint16_t radius;
  uint16_t angle;
};


float distances[360];
unsigned long times[360];

bool reading = false;
bool writing = false;

uint16_t getIndex(float angle);
void Task1Code(void * pvParameters);
float readDistance(uint16_t angle);

// put your setup code here, to run once:
void setup() {
  pinMode(PIN_LIDAR_MOTOR, OUTPUT);

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
    if (!writing) {
      reading = true;
      for (int i = 0; i < 360; i++) {
        Serial.println("Angle: " + String(i) + " , Distance: " + readDistance(i));
        delay(5);
      }
      reading = false;
      prev_ms = millis() + 25000;
    }
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

// Create code for task1
void Task1Code(void * pvParameters) {
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());

  for (;;) {
    if (reading);
    else {
      if (IS_OK(lidar.waitPoint())) {
        float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
        float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
        //bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan

        writing = true;
        //distances[uint16_t(angle)] = distance;
        uint16_t index = getIndex(angle);
        distances[index] = distance;
        times[index] = millis();
        writing = false;
      }
    }
  }
}

// Angle from 0 to 359
float readDistance(uint16_t angle) {
  //while (writing);
  //reading = true;
  float distanceMeasure = distances[angle];
  //reading = false;
  return distanceMeasure;
}