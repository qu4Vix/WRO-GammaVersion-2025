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
    inline Point2D getPoint(uint16_t index);
  private:
    uint16_t index = 0;
    uint16_t _size;
    Point2D * _cartesianData;
    polarPoint2D * _data;
};

extern int exeRANSAC(lidarStorage * storage, Line2D * outputBuffer, Point2D * unassocBuffer);

lidarStorage::lidarStorage(uint16_t size) : _size(size) {
  _data = new polarPoint2D[_size];
  _cartesianData = new Point2D[_size];
}

lidarStorage::~lidarStorage() {
  delete[] _data;
  delete[] _cartesianData;
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
    _cartesianData[i].x = _data[i].range * sin(_data[i].bearing * M_PI / 180);  // The angle is given with respect to the positive y axis, hence, angle=0 lies on the y positive axis
    _cartesianData[i].y = _data[i].range * cos(_data[i].bearing * M_PI / 180);  // So x = r * sin(0) and y = r * cos(0)
  }
  return _cartesianData;
}

inline polarPoint2D lidarStorage::getMeasurement(uint16_t index) {
  return _data[index];
}

inline Point2D lidarStorage::getPoint(u_int16_t index) {
  return _cartesianData[index];
}

#define SPIKE_JUMP 250
// Finds blocks from the full measurement array
void findBlocks(lidarStorage * dataStorage) {
  uint16_t spikes[dataStorage->getSize()];
  uint16_t spikeRecord = 0;
  for (uint16_t i = 0; i < dataStorage->getSize() - 1; i++) {
    if (abs(dataStorage->getMeasurement(i).range - dataStorage->getMeasurement(i+1).range) > SPIKE_JUMP) {
      if (abs(dataStorage->getMeasurement(i).bearing - dataStorage->getMeasurement(i+1).bearing) < 5) {
        spikes[spikeRecord] = i;
        spikes[spikeRecord + 1] = i + 1;
        spikeRecord += 2;
      }
    }
  }
  Point2D landmarks[spikeRecord];
  uint8_t landmark = 0;
  for (uint8_t j = 0; j < spikeRecord; j++) {
    if (abs(dataStorage->getPoint(j).x - dataStorage->getPoint(j+1).x) < 100) {
      if (abs(dataStorage->getPoint(j).y - dataStorage->getPoint(j+1).y) < 100) {
        landmarks[landmark] = Point2D{(dataStorage->getPoint(j).x + dataStorage->getPoint(j+1).x)/2,(dataStorage->getPoint(j).y + dataStorage->getPoint(j+1).y)/2};
      }
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