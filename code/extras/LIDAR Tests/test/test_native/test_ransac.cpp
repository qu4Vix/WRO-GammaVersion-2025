#include <unity.h>
#include <stdint.h>

#include "ransacAlgorithm.h"
#include "lidarHandling.h"

void test_perp_point_on_line() {
    // Line in the form y = ax + b
    Line2D line;
    line.a = 2.5;
    line.b = -3;
    // Point trough which the perpendicular line will pass trough
    Point2D point;
    point.x = -7;
    point.y = 7;
    // Expected result point (calculated graphically with DESMOS)
    Point2D expected;
    expected.x = 2.48276;
    expected.y = 3.2069;
    // Actual result given by the PerpPointOnLine function
    Point2D actual = PerpPointOnLine(line, point);

    // TEST
    TEST_ASSERT_EQUAL_FLOAT(expected.x, actual.x);
    TEST_ASSERT_EQUAL_FLOAT(expected.y, actual.y);    
}

void test_best_fit_line() {
    Line2D expectedLine;
    expectedLine.a = 2.5;
    expectedLine.b = -3;
    Point2D buffer[10];
    for (int i = 0; i < 10; i++) {
        buffer[i].x = 2*i+1;
        buffer[i].y = expectedLine.a * buffer[i].x + expectedLine.b;
    }
    Line2D resLine = CalcBestFitLine(buffer, (sizeof(buffer)/sizeof(Point2D)));
    TEST_ASSERT_EQUAL_FLOAT(expectedLine.a, resLine.a);
    TEST_ASSERT_EQUAL_FLOAT(expectedLine.b, resLine.b);
}

void test_ransac_extraction() {
    Line2D expectedLine;
    expectedLine.a = 2.5;
    expectedLine.b = -3;
    Point2D buffer[10];
    for (int i = 0; i < 10; i++) {
        buffer[i].x = 2*i+1;
        buffer[i].y = expectedLine.a * buffer[i].x + expectedLine.b;
    }
    Line2D resLine[5];
    TEST_ASSERT_EQUAL(1, exeRANSAC(buffer, (sizeof(buffer)/sizeof(Point2D)), resLine));
    TEST_ASSERT_EQUAL_FLOAT(expectedLine.a, resLine[0].a);
    TEST_ASSERT_EQUAL_FLOAT(expectedLine.b, resLine[0].b);
}

void test_lidar_conversion() {
    lidarStorage storage(10);
    double rawData[20] = {352.3865, 704.5, 353.7158, 703.3, 355.0891, 702.5, 356.4349, 702.5, 357.6215, 703.5,
                          358.9508, 704.0, 0.2637, 705.0, 1.7468, 706.3, 2.9993, 707.8, 4.4495, 709.3};
    for (int i = 0; i < 10; i++) {
        storage.addMeasurements(rawData[2*i+1], rawData[2*i]);
    }
    Point2D* buffer = storage.convertToCartesian();
    TEST_ASSERT_EQUAL(698.28937, buffer[0].x);
    TEST_ASSERT_EQUAL(-93.339160, buffer[0].y);
}

void test_lidar_line_ransac_extraction() {
    lidarStorage storage(10);
    double rawData[20] = {352.3865, 704.5, 353.7158, 703.3, 355.0891, 702.5, 356.4349, 702.5, 357.6215, 703.5,
                          358.9508, 704.0, 0.2637, 705.0, 1.7468, 706.3, 2.9993, 707.8, 4.4495, 709.3};
    for (int i = 0; i < 10; i++) {
        storage.addMeasurements(rawData[2*i+1], rawData[2*i]);
    }
    Point2D* buffer = storage.convertToCartesian();
    Line2D resLine[5];
    TEST_ASSERT_EQUAL(1, exeRANSAC(buffer, (sizeof(buffer)/sizeof(Point2D)), resLine));
    TEST_ASSERT_EQUAL_FLOAT(0, resLine[0].a);
    TEST_ASSERT_EQUAL_FLOAT(703, resLine[0].b);
}

void void_test() {
    // void test for comparing test timings
    TEST_ASSERT_EQUAL(true, true);
}

// Called before each test
void setUp() {

}

// Test
int main() {
    UNITY_BEGIN();
    //RUN_TEST(void_test);
    //RUN_TEST(test_perp_point_on_line);
    //RUN_TEST(test_best_fit_line);
    //RUN_TEST(test_ransac_extraction);
    //RUN_TEST(test_lidar_conversion);
    RUN_TEST(test_lidar_line_ransac_extraction);
    return UNITY_END();
}

// Called after each test
void tearDown() {

}