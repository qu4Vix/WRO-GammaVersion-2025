#ifndef LIDAR_HANDLE_h
#define LIDAR_HANDLE_h

//#include "Arduino.h"
//#include "RPLidar.h"
#include "ransacAlgorithm.h"

// Measurement of the lidar given by range and bearing
struct lidarMeasurement
{
  double radius;
  double angle;
};

class lidarStorage {
    public:
        lidarStorage(uint16_t size);
        ~lidarStorage();
        void addMeasurements(double radius, double angle);
        void addMeasurements(lidarMeasurement measurement);
        Point2D * convertToCartesian();
    private:
        Point2D * cartesianData;
        uint16_t index = 0;
        uint16_t _size;
        lidarMeasurement * _data;
};

lidarStorage::lidarStorage(uint16_t size) : _size(size) {
    _data = new lidarMeasurement[_size];
    cartesianData = new Point2D[_size];
}

lidarStorage::~lidarStorage() {
    delete[] _data;
}

void lidarStorage::addMeasurements(double radius, double angle) {
    _data[index].radius = radius;
    _data[index].angle = angle;
    index++;
}

void lidarStorage::addMeasurements(lidarMeasurement measurement) {
    _data[index] = measurement;
    index++;
}

Point2D * lidarStorage::convertToCartesian() {
  for (uint16_t i = 0; i < _size; i++) {
    cartesianData[i].x = _data[i].radius * sin(_data[i].angle * M_PI / 180);  // The angle is given with respect to the positive y axis, hence, angle=0 lies on the y positive axis
    cartesianData[i].y = _data[i].radius * cos(_data[i].angle * M_PI / 180);  // So x = r * sin(0) and y = r * cos(0)
  }
  return cartesianData;
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

/*
lidarStorage * pStorage;

// Create code for task1
void Task1Code(void * pvParameters) {
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());

  for (;;) {
    if (reading);
    else {
      if (IS_OK(lidar.waitPoint())) {
        uint16_t distance = lidar.getCurrentPoint().distance; //distance value in mm unit
        float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
        bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
        if (startBit) {
            pStorage = new lidarStorage(360);
        } else {
            
        }
      }
    }
  }
}
*/

#endif