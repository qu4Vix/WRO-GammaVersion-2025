#ifndef LIDAR_HANDLE_h
#define LIDAR_HANDLE_h

//#include "Arduino.h"
//#include "RPLidar.h"
#include "lidarDataTypes.h"
#include <cstring>

class lidarStorage {
  public:
    lidarStorage(uint16_t size);
    ~lidarStorage();
    void addMeasurements(double radius, double angle);
    void addMeasurements(polarPoint2D measurement);
    void setArray(polarPoint2D * array, uint16_t size);
    inline uint16_t getSize();
    Point2D * convertToCartesian();
    inline polarPoint2D getMeasurement(uint16_t index);
  private:
    Point2D * cartesianData;
    uint16_t index = 0;
    uint16_t _size;
    polarPoint2D * _data;
};

lidarStorage::lidarStorage(uint16_t size) : _size(size) {
  _data = new polarPoint2D[_size];
  cartesianData = new Point2D[_size];
}

lidarStorage::~lidarStorage() {
  delete[] _data;
  delete[] cartesianData;
}

void lidarStorage::addMeasurements(double radius, double angle) {
  _data[index].range = radius;
  _data[index].bearing = angle;
  index++;
}

void lidarStorage::addMeasurements(polarPoint2D measurement) {
  _data[index] = measurement;
  index++;
}

void lidarStorage::setArray(polarPoint2D * array, uint16_t size) {
  memcpy(_data, array, size * sizeof(polarPoint2D));
}

inline uint16_t lidarStorage::getSize() {
  return this->_size;
}

Point2D * lidarStorage::convertToCartesian() {
  for (uint16_t i = 0; i < _size; i++) {
    cartesianData[i].x = _data[i].range * sin(_data[i].bearing * M_PI / 180);  // The angle is given with respect to the positive y axis, hence, angle=0 lies on the y positive axis
    cartesianData[i].y = _data[i].range * cos(_data[i].bearing * M_PI / 180);  // So x = r * sin(0) and y = r * cos(0)
  }
  return cartesianData;
}

inline polarPoint2D lidarStorage::getMeasurement(uint16_t index) {
  return _data[index];
}

#define SPIKE_JUMP 250
void findBlocks(polarPoint2D * dataBuffer, uint8_t size) {
  uint8_t spikes[size];
  uint8_t spikeRecord = 0;
  for (uint8_t i = 0; i < size - 1; i++) {
    if (abs(dataBuffer[i].range - dataBuffer[i+i].range) < SPIKE_JUMP) {
      spikes[spikeRecord] = i;
    }
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