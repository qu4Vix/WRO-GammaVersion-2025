#ifndef RANSAC_h
#define RANSAC_h

#include <stdint.h>
#include <random>

#define MAX_TRIALS 10           // Maximum number of attempts to find lines
#define SAMPLE_SIZE 10          // Sample size of the initial line
#define DEGREE_RANGE 3          // Degree range (from initial reading) to sample from
#define DISTANCE_TOLERANCE 5    // Max distance an inlier may be from the line
#define CONCENSUS 8            // Number of inliers to accept a line
#define MAX_LANDMARKS 20        // Maximum number of landmarks we allow to be extracted


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

Point2D PerpPointOnLine(Line2D line, Point2D point) {
    Point2D result;
    result.x = (point.x + line.a*point.y - line.a*line.b)/(line.a*line.a + 1);
    result.y = (line.a * result.x + line.b);
    return result;
}

double Distance2D(Point2D p1, Point2D p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt((dx*dx) + (dy*dy));
}

double SquareDistance2D(Point2D p1, Point2D p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return ((dx*dx) + (dy*dy));
}

Line2D CalcBestFitLine(Point2D * dataBuffer, unsigned int size) {
    double sumX = 0;
    double sumXX = 0;
    double sumY = 0;
    double sumXY = 0;
    for (unsigned int i = 0; i < size; i++) {
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
 * Function that executes the RANSAC algorithm for 2D line extraction
 * @param dataBuffer Buffer with the data points
 * @param size The size of the dataBuffe
 * @param outputBuffer An output buffer for the exctracted lines
 * @return The number of lines extracted to the output buffer, or -1 if failed
 * 
 */
int exeRANSAC(Point2D * dataBuffer, unsigned int size, Line2D * outputBuffer) {
    // Initialize number of trials
    unsigned int totalTrials = 0;
    // Setup the random device
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0 + DEGREE_RANGE, size - 1 - DEGREE_RANGE); // we substract one from size in case the distribution includes the right limit
    // Enter main loop
    while (totalTrials < MAX_TRIALS) {
        // Generate the central point of the sample and declare the buffer for the randomly selected points
        unsigned int centrePoint = dist(gen);
        Point2D sampleBuffer[2*DEGREE_RANGE]; // ¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡ SHOULD BE SAMPLE_SIZE instead of 2*DEGREE RANGE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // Generate the random sample
        for (int i = -DEGREE_RANGE; i < DEGREE_RANGE; i++) {
            sampleBuffer[i] = dataBuffer[centrePoint+i];  // SHOULD BE RANDOMLY SELECTED    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }
        // Calculate the best fit line for this sample
        Line2D bestFitLine = CalcBestFitLine(sampleBuffer, sizeof(sampleBuffer)/sizeof(Point2D));
        // Check for points lying on the line
        unsigned int totalPointsLiying = 0;
        Point2D pointsLiying[size];
        for (unsigned int j = 0; j < size; j++) {
            if (SquareDistance2D(PerpPointOnLine(bestFitLine, dataBuffer[j]), dataBuffer[j]) < DISTANCE_TOLERANCE*DISTANCE_TOLERANCE) {
                pointsLiying[totalPointsLiying] = dataBuffer[j];
                totalPointsLiying++;
            }
        }
        if (totalPointsLiying > CONCENSUS) {
            Line2D extractedLine = CalcBestFitLine(pointsLiying, totalPointsLiying);
            outputBuffer[0] = extractedLine;
            return 1;
        }
        totalTrials++;
    }
    return -1; // return idea: return -1 if error, return data via a pointer to an array of MAX_LANDMARKS size, given as a parameter and return the size of the data as success
}

#endif