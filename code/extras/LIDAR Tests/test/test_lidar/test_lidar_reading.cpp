#include <Arduino.h>
#include <RPLidar.h>
#include <esp_task_wdt.h> 

#include "lidarHandling.h"

#define PIN_LIDAR_MOTOR 5 // The PWM pin for control the speed of RPLIDAR's motor.
#define PIN_LED_Red 12
#define PIN_LED_Blue 13
#define PIN_BUTTON 33

HardwareSerial lidarSerial(2);
RPLidar lidar;

TaskHandle_t Task1;

polarPoint2D lidarBuffer[365];
uint16_t measurementsIndex = 0;
lidarStorage * pStorage = new lidarStorage(1);

void Task1Code(void * pvParameters);

// put your setup code here, to run once:
void setup() {
  pinMode(PIN_LIDAR_MOTOR, OUTPUT);
  pinMode(PIN_LED_Red, OUTPUT);
  pinMode(PIN_LED_Blue, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  
  digitalWrite(PIN_LED_Blue, HIGH);
  while (digitalRead(PIN_BUTTON));
  digitalWrite(PIN_LED_Blue, LOW);

  Serial.begin(115200);
  Serial.println("Start");
  lidar.begin(lidarSerial);

  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) {Serial.println(info.model);
    delay(500);
  }
  Serial.println(info.model);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  Serial.println("info: " + String(health.status) +", " + String(health.error_code));
  // detected...
  while (!IS_OK(lidar.startScan())) {delay(500);};
  Serial.println("Lidar begin");
  analogWrite(PIN_LIDAR_MOTOR, 255);
  Serial.println("motor start");
  
  delay(500);
  Serial.println("Creating task");
  // Asign task 1 to core 0
  xTaskCreatePinnedToCore(
    Task1Code,
    "Task1",
    50000,
    NULL,
    1,
    &Task1,
    0);
  Serial.println("End setup");
}

// put your main code here, to run repeatedly:
void loop() {

}

// Create code for task1
void Task1Code(void * pvParameters) {
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());
  // whether the provisional storage is to be loaded to the main storage or not
  bool loadData = false;
  lidarStorage * provisionalStorage;
  static unsigned long prev_ms = millis();
  bool skipLoad = false;
  for (;;) {
    vTaskDelay(1);/*
    UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("Stack left: %d\n", stackLeft);*/
    
    // wait for new data point
    if (IS_OK(lidar.waitPoint())) {
      // get current point data
      double distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      double angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      bool startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      // if the point belongs to a new scan store the current buffer to the provisional storage
      if (startBit) {
        //Serial.println("StartBit");
        //Serial.printf("Storage size: %u\n", measurementsIndex);
        Serial.printf("Loop time: %u   ", millis()-prev_ms);
        Serial.printf("Measurements taken: %u\n", measurementsIndex);
        if (measurementsIndex > 10 && !skipLoad) {
          provisionalStorage = new lidarStorage(measurementsIndex);
          provisionalStorage->setArray(lidarBuffer, measurementsIndex);
          // indicate that the provisional storage is to be loaded
          loadData = true;
        }
        // reset the index to start storing the new measurements
        measurementsIndex = 0;
        skipLoad = false;
        prev_ms = millis();
      }
      if (loadData) {
        // do not load when reading to prevent corruption
          // delete the previous storage and store the provisional storage
          pStorage->~lidarStorage();
          pStorage = provisionalStorage;
          // data has been loaded
          loadData = false;
      }
      //Serial.printf("measurement: %u\n", measurementsIndex);
      // store the new measurement in the buffer (range and bearing, the angle between 180 and -180 for easy substractions)
      lidarBuffer[measurementsIndex] = polarPoint2D{distance, (angle > 180)?(angle - 360):angle};
      measurementsIndex++;
      if (measurementsIndex >= 365) {
        measurementsIndex = 0;
        skipLoad = true;
      }
    }
  }
}
