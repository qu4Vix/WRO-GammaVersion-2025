#include <Arduino.h>
#include <unity.h>

#include "ransacAlgorithm.h"
#include "lidarHandling.h"
#include "lidarData.h"

void test_lidar_ransac_from_file() {
    double * bufferChosen = lidarPoints3;
    lidarStorage storage(90);
    for (int i = 0; i < storage.getSize(); i++) {
        storage.addMeasurements(bufferChosen[2*i+1], (bufferChosen[2*i]>180)?(bufferChosen[2*i] - 360):bufferChosen[2*i]);
    }
    Line2D resLine[10];
    uint32_t prev_ms = millis();
    int8_t out = exeRANSAC(&storage, resLine, 0);
    printf("Time: %u ", millis()- prev_ms);
    for (uint8_t line = 0; line < out; line++) {
        printf("Line a: %lf ", resLine[line].a);
        printf("b: %lf \n", resLine[line].b);
    }
    TEST_ASSERT_EQUAL(3, out);
}

void test_cartesian_conversion() {
    double * bufferChosen = lidarPoints3;
    lidarStorage storage(90);
    for (int i = 0; i < storage.getSize(); i++) {
        storage.addMeasurements(bufferChosen[2*i+1], (bufferChosen[2*i]>180)?(bufferChosen[2*i] - 360):bufferChosen[2*i]);
    }
    printf("\n");
    uint32_t prev_ms = millis();
    storage.convertToCartesian();
    printf("Time: %u \n", millis()- prev_ms);
    TEST_ASSERT_EQUAL(3, 3);
}

// Called before each test
void setUp() {
    printf("Setting up test...\n");
}

// Test
void setup() {
    delay(1000);

    UNITY_BEGIN();
    RUN_TEST(test_lidar_ransac_from_file);
    RUN_TEST(test_cartesian_conversion);
    UNITY_END();
}

void loop() {

}

// Called after each test
void tearDown() {

}