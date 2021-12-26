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
    EXPECT_EQ(circleTwo.getRadius(), circleTwoRadius);
    EXPECT_EQ(circleThree.getRadius(), circleThreeRadius);
}

TEST_F(CircleTest, SetRadius) {
    circleOne.setRadius(2.1);
    EXPECT_EQ(circleOne.getRadius(), 2.1);
}

TEST_F(CircleTest, ScaleShape) {
    circleTwo.scaleShape(2.0);
    EXPECT_EQ(circleTwo.getRadius(), 2 * circleTwoRadius);
}

TEST_F(CircleTest, PrintParameters) { circleOne.printParameters(); }
