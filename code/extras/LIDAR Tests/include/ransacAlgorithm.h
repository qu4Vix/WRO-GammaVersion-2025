#ifndef RANSAC_h
#define RANSAC_h

#include <stdint.h>
#include <random>

#include "lidarDataTypes.h"
#include "lidarHandling.h"

#define Native false

#define MAX_TRIALS 10           // Maximum number of attempts to find lines             (uint8_t)(1 byte)(0-255)
#define SAMPLE_SIZE 10          // Sample size of the initial line                      (uint8_t)(1 byte)(0-255)
#define MIN_SAMPLE 5
#define DEGREE_RANGE 6          // Degree range (from initial reading) to sample from   (int8_t)(1 byte)(0-127)
#define DEGREES_PER_MEASUREMENT 1.5
#define DISTANCE_TOLERANCE 50   // Max distance an inlier may be from the line          (in mm)
#define CONCENSUS 8             // Number of inliers to accept a line                   (uint16_t)(2 bytes)(0-65535)
#define MAX_LANDMARKS 20        // Maximum number of landmarks we allow to be extracted (uint8_t)(1 byte)(0-255)


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
 * Function which calculates the distance between two points in a plane
 * @param Point1
 * @param Point2
 * @return The distance between the two points, in double type
 * 
 */
double Distance2D(Point2D p1, Point2D p2) {
    return sqrt(SquareDistance2D(p1, p2));
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

double DistanceToLine(Line2D line, Point2D point) {
    return Distance2D(PerpPointOnLine(line, point), point);
}

double SquareDistanceToLine(Line2D line, Point2D point) {
    return SquareDistance2D(PerpPointOnLine(line, point), point);
}

/*
 *
 * Function which calculates the line of best fit from a 2D point data set
 * @param dataBuffer The pointer to the buffer containing the set of 2D points
 * @param size The size of the input buffer
 * @return The 2D line of best fit of the point set
 * 
 */
Line2D CalcBestFitLine(Point2D * dataBuffer, uint16_t size) {
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
Line2D CalcBestFitLineFromSample(Point2D * dataBuffer, const List<uint16_t> & dataIndices, uint16_t * sampleIndices, uint16_t sampleSize) {
    double sumX = 0;
    double sumXX = 0;
    double sumY = 0;
    double sumXY = 0;
    for (uint16_t i = 0; i < sampleSize; i++) {
        // Exctract the point index from the dataIndices array
        uint16_t dataIndex = dataIndices[sampleIndices[i]];
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
int exeRANSAC(lidarStorage * storage, Line2D * outputBuffer, Point2D * unassocBuffer) {
    Point2D * dataBuffer = storage->convertToCartesian();
    // Initialize number of trials
    uint8_t totalTrials = 0;
    // Number of lines extracted to the output buffer
    uint8_t linesExtracted = 0;
    // The list of the data buffer indices that contain unassociated points
    List<uint16_t> unassocPoints;
    // At the begining all points are unassociated
    for (uint16_t i = 0; i < storage->getSize(); i++) {
        unassocPoints.append(i);
    }
    // Setup the random device
#if Native == true
    std::random_device rd;
    uint32_t seed = rd();
#else
    uint32_t seed = esp_random();
#endif
    std::mt19937 gen(seed);
    // Enter main loop
    while (totalTrials < MAX_TRIALS) {
        // Increment the number of trials so that continues don't have to
        totalTrials++;
        // Create a distribution to choose an index from the unassociated data
        std::uniform_int_distribution<> mainDist(0, unassocPoints.size() - 1); // we substract one from size because the distribution includes the right limit
        // Generate the central point of the sample and declare the buffer for the randomly selected points
        uint16_t centrePoint = mainDist(gen);
        double centreAngle = storage->getMeasurement(unassocPoints[centrePoint]).bearing;
        uint16_t sampleBuffer[SAMPLE_SIZE];
        sampleBuffer[0] = centrePoint;
        //printf("Centre %u  ", centrePoint);
        uint8_t sampledPoints = 1;
        std::uniform_int_distribution<> sampleDist(centrePoint - DEGREE_RANGE*DEGREES_PER_MEASUREMENT, centrePoint + DEGREE_RANGE*DEGREES_PER_MEASUREMENT);
        // Generate the random sample
        uint8_t sampleTrials = 0;
        while ((sampleTrials < 2*DEGREE_RANGE*DEGREES_PER_MEASUREMENT) && (sampledPoints < SAMPLE_SIZE))
        {
            // Generate a random index and constrain it between 0 and the number of unassociated points left
            int16_t index = sampleDist(gen);
            index = (index < 0)?(unassocPoints.size() + index):((index >= unassocPoints.size())?(index - unassocPoints.size()):index);
            // check if index is new
            bool isNew = true;
            for (uint8_t i = 0; i < sampledPoints; i++)
            {
                if (sampleBuffer[i] == index) {
                    isNew = false;
                    //printf("Repeated %u  ", unassocPoints[index]);
                    break;
                }
            }
            if (isNew) {
                if (abs(storage->getMeasurement(unassocPoints[index]).bearing - centreAngle) < DEGREE_RANGE) { // what if angle 1 is 359º and angle 2 is 1º, the angular distance should be 2º
                    // accept point
                    sampleBuffer[sampledPoints] = index;
                    sampledPoints++;
                    //printf("Sampled %u  ", unassocPoints[index]);
                } //else printf("Failed %u  ", unassocPoints[index]);
            }
            sampleTrials++;
        }
        //printf("\n");
        // If too few points were sampled, continue to the next iterarion
        if (sampledPoints < MIN_SAMPLE) {
            //printf("Skipped %u  \n\n", sampledPoints);
            continue;
        }
        // Calculate the best fit line for this sample
        Line2D bestFitLine = CalcBestFitLineFromSample(dataBuffer, unassocPoints, sampleBuffer, sampledPoints);
        // Check for points lying on the line
        uint16_t totalPointsLiying = 0;
        uint16_t pointsLiying[unassocPoints.size()]; // Array of the indices of the list unassocPoints which point lies on the line
        for (uint16_t j = 0; j < unassocPoints.size(); j++) {
            if (SquareDistanceToLine(bestFitLine, dataBuffer[unassocPoints[j]]) < DISTANCE_TOLERANCE*DISTANCE_TOLERANCE) {
                pointsLiying[totalPointsLiying] = j;    // When we pass this to CalcBestFitLineFromSample, it expects indices of dataBuffer- i.e. from the List unassocPoints-, not indices of the list. On the other hand, to pop the points we better know the indices of the list.
                totalPointsLiying++;
                //printf("Lying: %u  ", j);
            }
        }
        //printf("Points Lying: %u  \n", totalPointsLiying);
        if (totalPointsLiying > CONCENSUS) {
            Line2D extractedLine = CalcBestFitLineFromSample(dataBuffer, unassocPoints, pointsLiying, totalPointsLiying);
            outputBuffer[linesExtracted] = extractedLine;
            linesExtracted++;
            for (uint16_t p = 0; p < totalPointsLiying; p++) {
                //printf("Popped: %u  ", pointsLiying[p]);
                unassocPoints.pop(pointsLiying[p] - p); // When a point is popped, all indices after are shifted left, so they have to be substracted the amount of pops to compensate
            }
            //printf("\n\n");
            // If there are less unassociated points left than the concensus, we will be unable to read any more lines, so break the loop 
            if (unassocPoints.size() < CONCENSUS) {
                //printf("Breaking \n\n");
                if (unassocBuffer) {
                    for (uint8_t unPt = 0; unPt < unassocPoints.size(); unPt++) {
                        unassocBuffer[unPt] = dataBuffer[unassocPoints[unPt]];
                    }
                }
                break;
            }
        }
    }
    //printf("UnassocPoints: %u  \n", unassocPoints.size());
    // If we have exctracted any lines, return the number of them
    if (linesExtracted) {
        return linesExtracted;
    }
    return -1; // return idea: return -1 if error, return data via a pointer to an array of MAX_LANDMARKS size, given as a parameter and return the size of the data as success
}

#endif