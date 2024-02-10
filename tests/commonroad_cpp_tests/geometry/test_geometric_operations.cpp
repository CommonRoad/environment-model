#include "test_geometric_operations.h"
#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/geometry/geometric_operations.h"

#include <cmath>

void GeometricOperationsTest::SetUp() {}

TEST_F(GeometricOperationsTest, AddObjectDimensionsOneVertice) {
    std::vector<vertex> qVertices;
    std::vector<vertex> pVertices; // result
    vertex temp{1, 1};

    qVertices.push_back(temp);
    double length = 2;
    double width = 3;
    pVertices = geometric_operations::addObjectDimensionsRectangle(qVertices, length, width);
    bool rightResults{false};
    if (pVertices[0].x == 0 && pVertices[0].y == 2.5 && pVertices[1].x == 2 && pVertices[1].y == 2.5 &&
        pVertices[2].x == 2 && pVertices[2].y == -0.5 && pVertices[3].x == 0 && pVertices[3].y == -0.5) {
        rightResults = true;
    }
    EXPECT_TRUE(rightResults);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsOneVerticeDetailed) {
    std::vector<vertex> qVertices;
    std::vector<vertex> pVertices; // result
    vertex temp{0, 0};

    qVertices.push_back(temp);
    double length = 5.4864;
    double width = 1.6459;
    pVertices = geometric_operations::addObjectDimensionsRectangle(qVertices, length, width);
    bool rightResults{false};
    if (pVertices[0].x == -2.7432 && pVertices[0].y == 0.82295 && pVertices[1].x == 2.7432 &&
        pVertices[1].y == 0.82295 && pVertices[2].x == 2.7432 && pVertices[2].y == -0.82295 &&
        pVertices[3].x == -2.7432 && pVertices[3].y == -0.82295) {
        rightResults = true;
    }
    EXPECT_TRUE(rightResults);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsArbitraryVertices) {
    std::vector<vertex> qVertices;
    std::vector<vertex> pVertices; // result
    vertex temp{1, 2};
    qVertices.push_back(temp);
    temp.x = 2;
    temp.y = 2;
    qVertices.push_back(temp);
    temp.x = 2;
    temp.y = 1;
    qVertices.push_back(temp);
    temp.x = 1;
    temp.y = 1;
    qVertices.push_back(temp);
    double length = 2;
    double width = 3;
    pVertices = geometric_operations::addObjectDimensionsRectangle(qVertices, length, width);
    EXPECT_EQ(pVertices[0].x, 0);
    EXPECT_EQ(pVertices[0].y, 3.5);
    EXPECT_EQ(pVertices[1].x, 3);
    EXPECT_EQ(pVertices[1].y, 3.5);
    EXPECT_EQ(pVertices[2].x, 3);
    EXPECT_EQ(pVertices[2].y, -0.5);
    EXPECT_EQ(pVertices[3].x, 0);
    EXPECT_EQ(pVertices[3].y, -0.5);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsFourVertices) {
    std::vector<vertex> qVertices;
    std::vector<vertex> pVertices; // result
    vertex temp{1, 1};
    qVertices.push_back(temp);
    temp.x = 1;
    temp.y = 1.5;
    qVertices.push_back(temp);
    temp.x = 1;
    temp.y = 2;
    qVertices.push_back(temp);
    temp.x = 2;
    temp.y = 2;
    qVertices.push_back(temp);
    temp.x = 2;
    temp.y = 1;
    qVertices.push_back(temp);
    double length = 2;
    double width = 3;
    pVertices = geometric_operations::addObjectDimensionsRectangle(qVertices, length, width);
    EXPECT_EQ(pVertices[0].x, 0);
    EXPECT_EQ(pVertices[0].y, -0.5);
    EXPECT_EQ(pVertices[1].x, 0);
    EXPECT_EQ(pVertices[1].y, 3.5);
    EXPECT_EQ(pVertices[2].x, 3);
    EXPECT_EQ(pVertices[2].y, 3.5);
    EXPECT_EQ(pVertices[3].x, 3);
    EXPECT_EQ(pVertices[3].y, -0.5);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsArbitraryVerticesDetailed) {
    std::vector<vertex> qVertices;
    std::vector<vertex> pVertices; // result
    vertex temp{11.76174, 12.960000000000004};
    qVertices.push_back(temp);
    temp.x = 37.681740000000005;
    temp.y = 12.960000000000004;
    qVertices.push_back(temp);
    temp.x = 37.681740000000005;
    temp.y = -12.960000000000004;
    qVertices.push_back(temp);
    temp.x = 11.76174;
    temp.y = -12.960000000000004;
    qVertices.push_back(temp);
    double length = 5.334;
    double width = 1.7983;
    pVertices = geometric_operations::addObjectDimensionsRectangle(qVertices, length, width);
    EXPECT_EQ(pVertices[0].x, qVertices[0].x - 0.5 * length);
    EXPECT_EQ(pVertices[0].y, qVertices[0].y + 0.5 * width);
    EXPECT_EQ(pVertices[1].x, qVertices[1].x + 0.5 * length);
    EXPECT_EQ(pVertices[1].y, qVertices[1].y + 0.5 * width);
    EXPECT_EQ(pVertices[2].x, qVertices[2].x + 0.5 * length);
    EXPECT_EQ(pVertices[2].y, qVertices[2].y - 0.5 * width);
    EXPECT_EQ(pVertices[3].x, qVertices[3].x - 0.5 * length);
    EXPECT_EQ(pVertices[3].y, qVertices[3].y - 0.5 * width);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsSixVertices) {
    std::vector<vertex> qVertices;
    std::vector<vertex> pVertices; // result

    vertex temp{0, 0};
    qVertices.push_back(temp);
    temp.x = 1.992705435012611;
    temp.y = 0.04;
    qVertices.push_back(temp);
    temp.x = 2.03431;
    temp.y = 0.04;
    qVertices.push_back(temp);
    temp.x = 2.03431;
    temp.y = -0.04;
    qVertices.push_back(temp);
    temp.x = 1.992705435012611;
    temp.y = -0.04;
    qVertices.push_back(temp);
    temp.x = 0;
    temp.y = 0;
    qVertices.push_back(temp);
    double length = 5.4864;
    double width = 1.6459;
    pVertices = geometric_operations::addObjectDimensionsRectangle(qVertices, length, width);
    EXPECT_NEAR(pVertices[0].x, -2.7432, epsilon);
    EXPECT_NEAR(pVertices[0].y, 0.82295, epsilon);
    EXPECT_NEAR(pVertices[0].y, 0.82295, epsilon);
    EXPECT_NEAR(pVertices[1].x, -0.750494564987389, epsilon);
    EXPECT_NEAR(pVertices[1].y, 0.86295, epsilon);
    EXPECT_NEAR(pVertices[2].x, 4.777509999999999, epsilon);
    EXPECT_NEAR(pVertices[2].y, 0.86295, epsilon);
    EXPECT_NEAR(pVertices[3].x, 4.777509999999999, epsilon);
    EXPECT_NEAR(pVertices[3].y, -0.86295, epsilon);
    EXPECT_NEAR(pVertices[4].x, -0.750494564987389, epsilon);
    EXPECT_NEAR(pVertices[4].y, -0.86295, epsilon);
    EXPECT_NEAR(pVertices[5].x, -2.7432, epsilon);
    EXPECT_NEAR(pVertices[5].y, -0.82295, epsilon);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsCircle) {
    std::vector<vertex> pVertices;
    vertex center{0, 0};
    double radius = 1;
    pVertices = geometric_operations::addObjectDimensionsCircle(center, radius);
    bool rightResults{false};
    EXPECT_NEAR(pVertices[0].x, 0.707, 1e-2);
    EXPECT_NEAR(pVertices[0].y, 0.707, 1e-2);
    EXPECT_NEAR(pVertices[1].x, 0.0, 1e-2);
    EXPECT_NEAR(pVertices[1].y, 1.0, 1e-2);
    EXPECT_NEAR(pVertices[2].x, -0.707, 1e-2);
    EXPECT_NEAR(pVertices[2].y, 0.707, 1e-2);
    EXPECT_NEAR(pVertices[3].x, -1.0, 1e-2);
    EXPECT_NEAR(pVertices[3].y, 0.0, 1e-2);
    EXPECT_NEAR(pVertices[4].x, -0.707, 1e-2);
    EXPECT_NEAR(pVertices[4].y, -0.707, 1e-2);
    EXPECT_NEAR(pVertices[5].x, 0.0, 1e-2);
    EXPECT_NEAR(pVertices[5].y, -1.0, 1e-2);
    EXPECT_NEAR(pVertices[6].x, 0.707, 1e-2);
    EXPECT_NEAR(pVertices[6].y, -0.707, 1e-2);
    EXPECT_NEAR(pVertices[7].x, 1.0, 1e-2);
    EXPECT_NEAR(pVertices[7].y, 0.0, 1e-2);
}

TEST_F(GeometricOperationsTest, RotateAndTranslateVerticesRotate90Degree) {
    vertex translation{vertex{0, 0}};
    double rotationAngle{M_PI / 2};
    std::vector<vertex> initialVector{vertex{1, 1}};
    std::vector<vertex> expectedVector{vertex{-1, 1}};

    std::vector<vertex> result{
        geometric_operations::rotateAndTranslateVertices(initialVector, translation, rotationAngle)};

    EXPECT_NEAR(result.at(0).x, expectedVector.at(0).x, epsilon);
    EXPECT_NEAR(result.at(0).y, expectedVector.at(0).y, epsilon);
}

TEST_F(GeometricOperationsTest, ConstrainAngle) {
    EXPECT_NEAR(geometric_operations::constrainAngle(3 * M_PI), M_PI, 1e-9);
    EXPECT_NEAR(geometric_operations::constrainAngle(-3 * M_PI), -M_PI, 1e-9);
    EXPECT_NEAR(geometric_operations::constrainAngle(0), 0, 1e-9);
}

TEST_F(GeometricOperationsTest, RotateAndTranslateVerticesTranslate) {
    vertex translation{vertex{5.52, -2.2}};
    double rotationAngle{0};
    std::vector<vertex> initialVector{vertex{1, 1}};
    std::vector<vertex> expectedVector{vertex{6.52, -1.2}};

    std::vector<vertex> result{
        geometric_operations::rotateAndTranslateVertices(initialVector, translation, rotationAngle)};

    EXPECT_NEAR(result.at(0).x, expectedVector.at(0).x, epsilon);
    EXPECT_NEAR(result.at(0).y, expectedVector.at(0).y, epsilon);
}

TEST_F(GeometricOperationsTest, computeDistanceFromPolylines) {
    auto polylineA{std::vector<vertex>{vertex{0, 1}, vertex{1, 2}, vertex{2, 1.5}, vertex{3, 1.0}, vertex{4, 1.0},
                                       vertex{4.5, 5.0}}};
    auto polylineB{
        std::vector<vertex>{vertex{0, 0}, vertex{1, 0}, vertex{2, 0}, vertex{3, 0}, vertex{4, 0}, vertex{5, 0}}};
    auto polylineC{std::vector<vertex>{vertex{0, 0}}};
    auto polylineD{std::vector<vertex>{vertex{0, 0}, vertex{1, 0}}};
    auto result{geometric_operations::computeDistanceFromPolylines(polylineA, polylineB)};
    EXPECT_EQ(result[0], 1);
    EXPECT_EQ(result[1], 2);
    EXPECT_EQ(result[2], 1.5);
    EXPECT_EQ(result[3], 1);
    EXPECT_EQ(result[4], 1);
    EXPECT_NEAR(result[5], 5.02494, 0.0001);

    EXPECT_THROW(geometric_operations::computeDistanceFromPolylines(polylineA, polylineC), std::logic_error);
    EXPECT_THROW(geometric_operations::computeDistanceFromPolylines(polylineA, polylineD), std::logic_error);
}
