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
    circleFour = Circle(circleFourRadius, circleFourCenter);
}

TEST_F(CircleTest, Initialization) {
    EXPECT_EQ(circleOne.getType(), ShapeType::circle);
    EXPECT_EQ(circleOne.getRadius(), circleOneRadius);
    EXPECT_EQ(circleOne.getCenter().x, circleOneCenter.x);
    EXPECT_EQ(circleOne.getCenter().y, circleOneCenter.y);
    EXPECT_EQ(circleTwo.getRadius(), circleTwoRadius);
    EXPECT_EQ(circleTwo.getCenter().x, circleTwoCenter.x);
    EXPECT_EQ(circleTwo.getCenter().y, circleTwoCenter.y);
    EXPECT_EQ(circleThree.getRadius(), circleThreeRadius);
    EXPECT_EQ(circleFour.getRadius(), circleFourRadius);
    EXPECT_EQ(circleFour.getCenter().x, circleFourCenter.x);
    EXPECT_EQ(circleFour.getCenter().y, circleFourCenter.y);
}

TEST_F(CircleTest, SetRadius) {
    circleOne.setRadius(2.1);
    EXPECT_EQ(circleOne.getRadius(), 2.1);
}

TEST_F(CircleTest, ScaleShape) {
    circleTwo.scaleShape(2.0);
    EXPECT_EQ(circleTwo.getRadius(), 2 * circleTwoRadius);
}

TEST_F(CircleTest, SetCenter) {
    circleThree.setCenter(3, 4);
    EXPECT_EQ(circleThree.getCenter().x, 3);
    EXPECT_EQ(circleThree.getCenter().y, 4);
}

TEST_F(CircleTest, PrintParameters) { circleOne.printParameters(); }

TEST_F(CircleTest, GetCircumradius) {
    EXPECT_EQ(circleOne.getCircumradius(), circleOne.getRadius());
    EXPECT_EQ(circleTwo.getCircumradius(), circleTwo.getRadius());
    EXPECT_EQ(circleThree.getCircumradius(), circleThree.getRadius());
    EXPECT_EQ(circleFour.getCircumradius(), circleFour.getRadius());
}
