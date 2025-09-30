#ifndef STATISTICS_h
#define STATISTICS_h

double average(double * dataBuffer, unsigned int size) {
    double totalSum = 0;
    for (unsigned int i = 0; i < size; i++) {
        totalSum += dataBuffer[i];
    }
    return totalSum / size;
}

double covariance(double * dataBuffer, unsigned int size) {
    double totalSum2 = 0;
    for (unsigned int i = 0; i < size; i++) {
        totalSum2 += dataBuffer[i]*dataBuffer[i];
    }
    return totalSum2 / size;
}

#endif