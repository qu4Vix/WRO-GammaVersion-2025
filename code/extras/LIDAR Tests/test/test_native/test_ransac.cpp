#include <unity.h>
#include <stdint.h>

#include "ransacAlgorithm.h"
#include "lidarHandling.h"
#include "lidarData.h"

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
    //TEST_ASSERT_EQUAL(1, exeRANSAC(buffer, (sizeof(buffer)/sizeof(Point2D)), resLine));
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
    TEST_ASSERT_EQUAL(698.28937, buffer[0].y);
    TEST_ASSERT_EQUAL(-93.339160, buffer[0].x);
}

void test_lidar_conversion_best_fit() {
    lidarStorage storage(10);
    double rawData[20] = /*{352.3865, 704.5, 353.7158, 703.3, 355.0891, 702.5, 356.4349, 702.5, 357.6215, 703.5,
                          358.9508, 704.0, 0.2637, 705.0, 1.7468, 706.3, 2.9993, 707.8, 4.4495, 709.3};*/
                          {55.1074, 457.8, 56.3708, 450.5, 57.9639, 443.0, 59.3427, 437.0, 60.6665, 432.5,
                          62.1826, 428.0, 63.6053, 423.0, 64.6051, 418.3, 66.0278, 414.5, 67.7142, 410.8};
    for (int i = 0; i < 10; i++) {
        storage.addMeasurements(rawData[2*i+1], rawData[2*i]);
    }
    Point2D* buffer = storage.convertToCartesian();
    Line2D bestFit = CalcBestFitLine(buffer, 10);
    TEST_ASSERT_EQUAL(-18, bestFit.a);
    TEST_ASSERT_EQUAL(7319, bestFit.b);
    TEST_ASSERT_EQUAL(0, DistanceToLine(bestFit, Point2D{0,0}));
}

void test_lists() {
    std::vector<uint8_t> v = {1,3,6,10,15,21};

    v.erase(v.begin());
    v.erase(v.begin()+2);

    TEST_ASSERT_EQUAL(3, v.at(0));
    TEST_ASSERT_EQUAL(15, v.at(2));
}

void test_list_class() {
    List<uint8_t> v;
    uint8_t array[5] = {1,3,6,10,15};
    v.assignArray(array, 5);
    v.pop(3);
    v.append(21);
    TEST_ASSERT_EQUAL(15, v[3]);
    TEST_ASSERT_EQUAL(21, v[4]);
    TEST_ASSERT_EQUAL(5, v.size());
}

void test_lidar_line_ransac_extraction() {
    lidarStorage storage(20);
    double rawData[40] = {352.3865, 704.5, 353.7158, 703.3, 355.0891, 702.5, 356.4349, 702.5, 357.6215, 703.5,
                          358.9508, 704.0, 0.2637, 705.0, 1.7468, 706.3, 2.9993, 707.8, 4.4495, 709.3,
                          55.1074, 457.8, 56.3708, 450.5, 57.9639, 443.0, 59.3427, 437.0, 60.6665, 432.5,
                          62.1826, 428.0, 63.6053, 423.0, 64.6051, 418.3, 66.0278, 414.5, 67.7142, 410.8};
    for (int i = 0; i < 20; i++) {
        storage.addMeasurements(rawData[2*i+1], (rawData[2*i]>180)?(rawData[2*i] - 360):rawData[2*i]);
    }
    Line2D resLine[10];
    int8_t out = exeRANSAC(&storage, resLine);
    printf("Line 0 a: %lf ", resLine[0].a);
    printf("b: %lf \n", resLine[0].b);
    printf("Line 1 a: %lf ", resLine[1].a);
    printf("b: %lf \n", resLine[1].b);
    TEST_ASSERT_EQUAL(2, out);
    //TEST_ASSERT_EQUAL(0, resLine[0].a);
    //TEST_ASSERT_EQUAL(704, resLine[0].b);
    //TEST_ASSERT_EQUAL(-18, resLine[0].a);
    //TEST_ASSERT_EQUAL(7319, resLine[0].b);
}

void test_lidar_ransac_from_file() {
    double * bufferChosen = lidarPoints3;
    lidarStorage storage(90);
    for (int i = 0; i < storage.getSize(); i++) {
        storage.addMeasurements(bufferChosen[2*i+1], (bufferChosen[2*i]>180)?(bufferChosen[2*i] - 360):bufferChosen[2*i]);
    }
    Line2D resLine[10];
    int8_t out = exeRANSAC(&storage, resLine);
    for (uint8_t line = 0; line < out; line++) {
        printf("Line a: %lf ", resLine[line].a);
        printf("b: %lf \n", resLine[line].b);
    }
    TEST_ASSERT_EQUAL(3, out);
}

void test_random_sampling() {
    uint8_t angles[50] = {  0,1,2,3,4,5,6,7,8,9,
                            15,16,17,18,19,20,21,22,23,24,
                            30,31,32,33,34,35,36,37,38,39,
                            45,46,47,48,49,50,51,52,53,54,
                            60,61,62,63,64,65,66,67,68,69};

    // Setup the random device
    std::random_device rd;
    std::mt19937 gen(rd());
    // Create a distribution to choose an index from the unassociated data
    std::uniform_int_distribution<> mainDist(0, 49);
    int8_t centreIndex = mainDist(gen);
    printf("Index %u  ", centreIndex);
    uint8_t centreAngle = angles[centreIndex];
    int8_t sampleBuffer[10] = {centreIndex,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    std::uniform_int_distribution<> sampleDist(centreIndex - 6, centreIndex + 6);
    for (uint8_t i = 1; i < 10; i++) {
        int8_t index = sampleDist(gen);
        index = (index<0)?(50 + index):((index>=50)?(index-50):index);
        bool isNew = true;
        for (uint8_t j = 0; j < i; j++) {
            if (sampleBuffer[j]==index) {
                isNew = false;
                printf("Repeated %u  ", index);
                break;
            }
        }
        if (isNew) {
            if (abs(angles[index] - centreAngle) < 5) {
                sampleBuffer[i] = index;
                printf("Index %u  ", index);
            } else printf("Failed %u  ", index);
        }
            
    }
    TEST_ASSERT_EQUAL(1,1);
}

void test_ransac_sampling() {
    lidarStorage storage(20);
    double rawData[40] = {352.3865, 704.5, 353.7158, 703.3, 355.0891, 702.5, 356.4349, 702.5, 357.6215, 703.5,
                          358.9508, 704.0, 0.2637, 705.0, 1.7468, 706.3, 2.9993, 707.8, 4.4495, 709.3,
                          55.1074, 457.8, 56.3708, 450.5, 57.9639, 443.0, 59.3427, 437.0, 60.6665, 432.5,
                          62.1826, 428.0, 63.6053, 423.0, 64.6051, 418.3, 66.0278, 414.5, 67.7142, 410.8};
    for (int i = 0; i < 20; i++) {
        storage.addMeasurements(rawData[2*i+1], (rawData[2*i]>180)?(rawData[2*i] - 360):rawData[2*i]);
    }
    // Setup the random device
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> mainDist(0, storage.getSize() - 1); // we substract one from size in case the distribution includes the right limit
    // Generate the central point of the sample and declare the buffer for the randomly selected points
    uint16_t centrePoint = mainDist(gen);
    printf("Centre %u  ", centrePoint);
    double centreAngle = storage.getMeasurement(centrePoint).angle;
    uint16_t sampleBuffer[SAMPLE_SIZE];
    sampleBuffer[0] = centrePoint;
    uint8_t sampledPoints = 1;
    std::uniform_int_distribution<> sampleDist(centrePoint - DEGREE_RANGE*DEGREES_PER_MEASUREMENT, centrePoint + DEGREE_RANGE*DEGREES_PER_MEASUREMENT);
    // Generate the random sample
    uint8_t sampleTrials;
    while ((sampleTrials < 2*DEGREE_RANGE*DEGREES_PER_MEASUREMENT) && (sampledPoints < SAMPLE_SIZE))
    {
        // Generate a random index and constrain it between 0 and the size of the storage
        int16_t index = sampleDist(gen);
        index = (index < 0)?(storage.getSize() + index):(index >= storage.getSize()?(index - storage.getSize()):index);
        // check if index is new
        bool isNew = true;
        for (uint8_t i = 0; i < sampledPoints; i++)
        {
            if (sampleBuffer[i] == index) {
                isNew = false;
                printf("Repeated %u  ", index);
                break;
            }
        }
        if (isNew) {
            if (abs(storage.getMeasurement(index).angle - centreAngle) < DEGREE_RANGE) { // what if angle 1 is 359º and angle 2 is 1º, the angular distance should be 2º
                // accept point
                sampleBuffer[sampledPoints] = index;
                sampledPoints++;
                printf("Index %u  ", index);
            } else printf("Failed %u  ", index);
        }
        sampleTrials++;
    }
    TEST_ASSERT_EQUAL(SAMPLE_SIZE, sampledPoints);
}

void test_continue_statement() {
    uint8_t sample[6] = {2,3,4,5,6,7};
    while (false) {
        for (uint8_t i = 0; i < 6; i++)
        {
            if (sample[i]==5) {
                break;
            }
        }
    }
    TEST_ASSERT_EQUAL(1, 1);
}

void void_test() {
    // void test for comparing test timings
    TEST_ASSERT_EQUAL(true, true);
}

// Called before each test
void setUp() {
    printf("Setting up test...\n");
}

// Test
int main() {
    UNITY_BEGIN();
    //RUN_TEST(void_test);
    //RUN_TEST(test_perp_point_on_line);
    //RUN_TEST(test_best_fit_line);
    //RUN_TEST(test_ransac_extraction);
    //RUN_TEST(test_lidar_conversion);
    //RUN_TEST(test_lidar_conversion_best_fit);
    //RUN_TEST(test_lidar_line_ransac_extraction);
    //RUN_TEST(test_lists);
    //RUN_TEST(test_list_class);
    //RUN_TEST(test_random_sampling);
    //RUN_TEST(test_ransac_sampling);
    //RUN_TEST(test_continue_statement);
    RUN_TEST(test_lidar_ransac_from_file);
    return UNITY_END();
}

// Called after each test
void tearDown() {

}