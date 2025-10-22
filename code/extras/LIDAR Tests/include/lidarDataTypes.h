#ifndef LIDAR_DATA_TYPES_h
#define LIDAR_DATA_TYPES_h

#include <vector>

// Measurement of the lidar given by range and bearing
struct lidarMeasurement
{
    double radius;
    double angle;
};

// Point on the 2D plane
struct Point2D
{
    double x;
    double y;
};

// y = ax + b form 
struct Line2D
{
    double a;
    double b;
};

template <typename T>
class List {
    private:
    std::vector<T> _data;
    public:
    void pop(size_t index){
        _data.erase(_data.begin() + index);
    }
    void assignArray(T * data, size_t size) {
        _data.assign(data, data + size);
    }
    void append(T datum) {
        _data.push_back(datum);
    }
    size_t size() const {
        return _data.size();
    }
    T& operator[](size_t index) {
        return _data[index];
    }
    const T& operator[](size_t index) const {
        return _data[index];
    }
};

#endif