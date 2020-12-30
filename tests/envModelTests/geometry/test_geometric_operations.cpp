//
// Created by sebastian on 26.12.20.
//

#include "test_geometric_operations.h"
#include "auxiliaryDefs/structs.h"

#include <cmath>

void GeometricOperationsTest::SetUp() {

}

TEST_F(GeometricOperationsTest, AddObjectDimensionsOneVertice){
    std::vector<vertex> q;
    std::vector<vertex> p; // result
    vertex temp{1, 1 };

    q.push_back(temp);
    double length = 2;
    double width = 3;
    p = addObjectDimensions(q, length, width);
    bool rightResults{ false };
    if (p[0].x == 0 && p[0].y == 2.5 && p[1].x == 2 && p[1].y == 2.5 && p[2].x == 2 && p[2].y == -0.5 && p[3].x == 0 &&
        p[3].y == -0.5) {
        rightResults = true;
    }
    EXPECT_TRUE(rightResults);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsOneVerticeDetailed){
    std::vector<vertex> q;
    std::vector<vertex> p; // result
    vertex temp{0, 0 };

    q.push_back(temp);
    double length = 5.4864;
    double width = 1.6459;
    p = addObjectDimensions(q, length, width);
    bool rightResults{ false };
    if (p[0].x == -2.7432 && p[0].y == 0.82295 && p[1].x == 2.7432 && p[1].y == 0.82295 && p[2].x == 2.7432 &&
        p[2].y == -0.82295 && p[3].x == -2.7432 && p[3].y == -0.82295) {
        rightResults = true;
    }
    EXPECT_TRUE(rightResults);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsArbitraryVertices){
    std::vector<vertex> q;
    std::vector<vertex> p; // result
    vertex temp{1, 2 };
    q.push_back(temp);
    temp.x = 2;
    temp.y = 2;
    q.push_back(temp);
    temp.x = 2;
    temp.y = 1;
    q.push_back(temp);
    temp.x = 1;
    temp.y = 1;
    q.push_back(temp);
    double length = 2;
    double width = 3;
    p = addObjectDimensions(q, length, width);

    EXPECT_EQ(p[0].x, 0);
    EXPECT_EQ(p[0].y, 3.5);
    EXPECT_EQ(p[1].x, 3);
    EXPECT_EQ(p[1].y, 3.5);
    EXPECT_EQ(p[2].x, 3);
    EXPECT_EQ(p[2].y, -0.5);
    EXPECT_EQ(p[3].x, 0);
    EXPECT_EQ(p[3].y, -0.5);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsFourVertices){
    std::vector<vertex> q;
    std::vector<vertex> p; // result
    vertex temp{1, 1};
    q.push_back(temp);
    temp.x = 1;
    temp.y = 1.5;
    q.push_back(temp);
    temp.x = 1;
    temp.y = 2;
    q.push_back(temp);
    temp.x = 2;
    temp.y = 2;
    q.push_back(temp);
    temp.x = 2;
    temp.y = 1;
    q.push_back(temp);
    double length = 2;
    double width = 3;
    p = addObjectDimensions(q, length, width);
    EXPECT_EQ(p[0].x, 0);
    EXPECT_EQ(p[0].y, -0.5);
    EXPECT_EQ(p[1].x, 0);
    EXPECT_EQ(p[1].y, 3.5);
    EXPECT_EQ(p[2].x, 3);
    EXPECT_EQ(p[2].y, 3.5);
    EXPECT_EQ(p[3].x, 3);
    EXPECT_EQ(p[3].y, -0.5);
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsArbitraryVerticesDetailed){
    std::vector<vertex> q;
    std::vector<vertex> p; // result
    vertex temp{11.76174, 12.960000000000004 };
    q.push_back(temp);
    temp.x = 37.681740000000005;
    temp.y = 12.960000000000004;
    q.push_back(temp);
    temp.x = 37.681740000000005;
    temp.y = -12.960000000000004;
    q.push_back(temp);
    temp.x = 11.76174;
    temp.y = -12.960000000000004;
    q.push_back(temp);
    double length = 5.334;
    double width = 1.7983;
    p = addObjectDimensions(q, length, width);
    // TODO use result
}

TEST_F(GeometricOperationsTest, AddObjectDimensionsSixVertices){
    std::vector<vertex> q;
    std::vector<vertex> p; // result

    vertex temp{0 , 0 };
    q.push_back(temp);
    temp.x = 1.992705435012611;
    temp.y = 0.04;
    q.push_back(temp);
    temp.x = 2.03431;
    temp.y = 0.04;
    q.push_back(temp);
    temp.x = 2.03431;
    temp.y = -0.04;
    q.push_back(temp);
    temp.x = 1.992705435012611;
    temp.y = -0.04;
    q.push_back(temp);
    temp.x = 0;
    temp.y = 0;
    q.push_back(temp);
    double length = 5.4864;
    double width = 1.6459;
    p = addObjectDimensions(q, length, width);
    EXPECT_NEAR(p[0].x, -2.7432, epsilon);
    EXPECT_NEAR(p[0].y, 0.82295, epsilon);
    EXPECT_NEAR(p[0].y, 0.82295, epsilon);
    EXPECT_NEAR(p[1].x, -0.750494564987389, epsilon);
    EXPECT_NEAR(p[1].y, 0.86295, epsilon);
    EXPECT_NEAR(p[2].x, 4.777509999999999, epsilon);
    EXPECT_NEAR(p[2].y, 0.86295, epsilon);
    EXPECT_NEAR(p[3].x, 4.777509999999999, epsilon);
    EXPECT_NEAR(p[3].y, -0.86295, epsilon);
    EXPECT_NEAR(p[4].x, -0.750494564987389, epsilon);
    EXPECT_NEAR(p[4].y, -0.86295, epsilon);
    EXPECT_NEAR(p[5].x, -2.7432, epsilon);
    EXPECT_NEAR(p[5].y, -0.82295, epsilon);
}

TEST_F(GeometricOperationsTest, RotateAndTranslateVerticesRotate90Degree){
    vertex translation {vertex{0, 0} };
    double rotationAngle { M_PI/2 };
    std::vector<vertex> initialVector {vertex{1, 1} };
    std::vector<vertex> expectedVector {vertex{-1, 1} };

    std::vector<vertex> result {rotateAndTranslateVertices(initialVector, translation, rotationAngle) };

    EXPECT_NEAR(result.at(0).x, expectedVector.at(0).x, epsilon);
    EXPECT_NEAR(result.at(0).y, expectedVector.at(0).y, epsilon);
}

TEST_F(GeometricOperationsTest, RotateAndTranslateVerticesTranslate){
    vertex translation {vertex{5.52, -2.2 } };
    double rotationAngle { 0 };
    std::vector<vertex> initialVector {vertex{1, 1 } };
    std::vector<vertex> expectedVector {vertex{6.52, -1.2 } };

    std::vector<vertex> result {rotateAndTranslateVertices(initialVector, translation, rotationAngle) };

    EXPECT_NEAR(result.at(0).x, expectedVector.at(0).x, epsilon);
    EXPECT_NEAR(result.at(0).y, expectedVector.at(0).y, epsilon);

}
