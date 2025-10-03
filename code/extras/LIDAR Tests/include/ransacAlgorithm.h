#ifndef RANSAC_h
#define RANSAC_h

#include <stdint.h>
#include <random>

#define MAX_TRIALS 10           // Maximum number of attempts to find lines             (uint8_t)(1 byte)(0-255)
#define SAMPLE_SIZE 10          // Sample size of the initial line
#define DEGREE_RANGE 3          // Degree range (from initial reading) to sample from   (int8_t)(1 byte)(0-127)
#define DISTANCE_TOLERANCE 10   // Max distance an inlier may be from the line          (in mm)
#define CONCENSUS 8             // Number of inliers to accept a line                   (uint16_t)(2 bytes)(0-65535)
#define MAX_LANDMARKS 20        // Maximum number of landmarks we allow to be extracted (uint8_t)(1 byte)(0-255)


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
    T operator[](size_t index) {
        return _data[index];
    }
};

/*
 *
 * Function which calculates the distance between two points in a plane
 * @param Point1
 * @param Point2
 * @return The distance between the two points, in double type
 * 
 */
double Distance2D(Point2D p1, Point2D p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt((dx*dx) + (dy*dy));
}

/*
 *
 * Function which calculates the squared distance between two points in a plane, for less computation time
 * @param Point1
 * @param Point2
 * @return The squared distance between the two points, in double type
 * 
 */
double SquareDistance2D(Point2D p1, Point2D p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return ((dx*dx) + (dy*dy));
}

/*
 *
 * Function which calculates the intersection point between the line and the line perpendicular to it which contains the point
 * @param Line
 * @param Point
 * @return The 2D point on the line which is closest to the point given
 * 
 */
Point2D PerpPointOnLine(Line2D line, Point2D point) {
    Point2D result;
    result.x = (point.x + line.a*point.y - line.a*line.b)/(line.a*line.a + 1);
    result.y = (line.a * result.x + line.b);
    return result;
}

/*
 *
 * Function which calculates the line of best fit from a 2D point data set
 * @param dataBuffer The pointer to the buffer containing the set of 2D points
 * @param size The size of the input buffer
 * @return The 2D line of best fit of the point set
 * 
 */
Line2D CalcBestFitLine(Point2D * dataBuffer, unsigned int size) {
    double sumX = 0;
    double sumXX = 0;
    double sumY = 0;
    double sumXY = 0;
    for (uint16_t i = 0; i < size; i++) {
        sumX += dataBuffer[i].x;
        sumXX += dataBuffer[i].x * dataBuffer[i].x;
        sumY += dataBuffer[i].y;
        sumXY += dataBuffer[i].x * dataBuffer[i].y;
    }
    Line2D bestFitLine;
    bestFitLine.a = (size*sumXY - sumX*sumY)/(size*sumXX - sumX*sumX);
    bestFitLine.b = (sumY - bestFitLine.a * sumX)/size;
    return bestFitLine;
}

/*
 *
 * Function which calculates the line of best fit from a 2D point data set
 * @param dataBuffer The pointer to the buffer containing the set of 2D points
 * @param dataIndices Indices of the points from the dataBuffer array that belong to the sample
 * @param size The size of the sample
 * @return The 2D line of best fit of the point set
 * 
 */
Line2D CalcBestFitLineFromSample(Point2D * dataBuffer, uint16_t * dataIndices, uint16_t sampleSize) {
    double sumX = 0;
    double sumXX = 0;
    double sumY = 0;
    double sumXY = 0;
    for (uint16_t i = 0; i < sampleSize; i++) {
        // Exctract the point index from the dataIndices array
        uint16_t dataIndex = dataIndices[i];
        sumX += dataBuffer[dataIndex].x;
        sumXX += dataBuffer[dataIndex].x * dataBuffer[dataIndex].x;
        sumY += dataBuffer[dataIndex].y;
        sumXY += dataBuffer[dataIndex].x * dataBuffer[dataIndex].y;
    }
    Line2D bestFitLine;
    bestFitLine.a = (sampleSize*sumXY - sumX*sumY)/(sampleSize*sumXX - sumX*sumX);
    bestFitLine.b = (sumY - bestFitLine.a * sumX)/sampleSize;
    return bestFitLine;
}

/*
 *
 * Function that executes the RANSAC algorithm for 2D line extraction
 * @param dataBuffer Buffer with the data points
 * @param size The size of the dataBuffe
 * @param outputBuffer An output buffer for the exctracted lines
 * @return The number of lines extracted to the output buffer, or -1 if failed to find lines
 * 
 */
int exeRANSAC(Point2D * dataBuffer, uint16_t size, Line2D * outputBuffer) {
    // Initialize number of trials
    uint8_t totalTrials = 0;
    // Number of lines extracted to the output buffer
    uint8_t linesExtracted = 0;
    // The list of the data buffer indices that contain unassociated points
    List<uint16_t> unassocPoints;
    // At the begining all points are unassociated
    for (uint16_t i = 0; i < size; i++) {
        unassocPoints.append(i);
    }
    // Setup the random device
    std::random_device rd;
    std::mt19937 gen(rd());
    // Enter main loop
    while (totalTrials < MAX_TRIALS) {
        // Create a distribution to choose an index from the unassociated data
        std::uniform_int_distribution<> dist(0 + DEGREE_RANGE, unassocPoints.size() - 1 - DEGREE_RANGE); // we substract one from size in case the distribution includes the right limit
        // Generate the central point of the sample and declare the buffer for the randomly selected points
        uint16_t centrePoint = dist(gen);
        uint16_t sampleBuffer[2*DEGREE_RANGE]; // ¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡ SHOULD BE SAMPLE_SIZE instead of 2*DEGREE RANGE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // Generate the random sample
        uint8_t index=0;
        for (int8_t i = -DEGREE_RANGE; i < DEGREE_RANGE; i++) {
            sampleBuffer[index] = centrePoint+i;  // ¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡ SHOULD BE RANDOMLY SELECTED    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            index++;
        }
        // Calculate the best fit line for this sample
        Line2D bestFitLine = CalcBestFitLineFromSample(dataBuffer, sampleBuffer, 2*DEGREE_RANGE);   // ¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡ SHOULD BE SAMPLE_SIZE instead of 2*DEGREE RANGE !!!!!!!!!!!!!!!!!!!!!
        // Check for points lying on the line
        uint16_t totalPointsLiying = 0;
        uint16_t pointsLiying[unassocPoints.size()];
        for (uint16_t j = 0; j < unassocPoints.size(); j++) {
            if (SquareDistance2D(PerpPointOnLine(bestFitLine, dataBuffer[j]), dataBuffer[j]) < DISTANCE_TOLERANCE*DISTANCE_TOLERANCE) {
                pointsLiying[totalPointsLiying] = j;
                totalPointsLiying++;
            }
        }
        if (totalPointsLiying > CONCENSUS) {
            Line2D extractedLine = CalcBestFitLineFromSample(dataBuffer, pointsLiying, totalPointsLiying);
            outputBuffer[linesExtracted] = extractedLine;
            linesExtracted++;
            for (uint16_t p = 0; p < unassocPoints.size(); p++) {
                for (uint16_t k = 0; k < totalPointsLiying; k++) {
                    if (pointsLiying[k] == p) {
                        unassocPoints.pop(p);
                    }
                }
            }
            // If there are less unassociated points left than the concensus, we will be unable to read any more lines, so break the loop 
            if (unassocPoints.size() < CONCENSUS) break;
        }
        totalTrials++;
    }
    // If we have exctracted any lines, return the number of them
    if (linesExtracted) {
        return linesExtracted;
    }
    return -1; // return idea: return -1 if error, return data via a pointer to an array of MAX_LANDMARKS size, given as a parameter and return the size of the data as success
}

#endif