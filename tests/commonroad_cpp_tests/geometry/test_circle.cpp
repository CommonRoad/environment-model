//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_circle.h"

void CircleTest::SetUp() {
    circleOne = Circle();
    circleTwo = Circle(circleTwoRadius);
    circleThree = Circle(circleThreeRadius);
}

TEST_F(CircleTest, Initialization) {
    EXPECT_EQ(circleOne.getType(), ShapeType::circle);
    EXPECT_EQ(circleOne.getRadius(), circleOneRadius);
    EXPECT_EQ(circleTwo.getLength(), circleTwoRadius);
    EXPECT_EQ(circleThree.getLength(), circleThreeRadius);
}

TEST_F(CircleTest, SetWidth) {
    circleOne.setWidth(2.1);
    EXPECT_EQ(circleOne.getWidth(), 2.1);
}

TEST_F(CircleTest, SetLength) {
    circleOne.setLength(2.1);
    EXPECT_EQ(circleOne.getLength(), 2.1);
}

TEST_F(CircleTest, ScaleShape) {
    circleTwo.scaleShape(2.0);
    EXPECT_EQ(circleTwo.getRadius(), 2 * circleTwoRadius);
}

TEST_F(CircleTest, PrintParameters) { circleOne.printParameters(); }
