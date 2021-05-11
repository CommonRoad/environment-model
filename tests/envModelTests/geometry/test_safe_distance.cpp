//
// Created by sebastian on 26.12.20.
//

#include "test_circle.h"

void CircleTestInitialization::setUpCircle() {
    circleOne = Circle();
    circleTwo = Circle(circleTwoRadius);
    circleThree = Circle(circleThreeRadius, circleThreeCenter);
    circleFour = Circle(circleFourRadius, circleFourCenter);
}

void CircleTest::SetUp() { setUpCircle(); }

TEST_F(CircleTest, Initialization) {
    EXPECT_EQ(circleOne.getType(), ShapeType::circle);
    EXPECT_EQ(circleOne.getRadius(), circleOneRadius);
    EXPECT_EQ(circleOne.getCenter().x, circleOneCenter.x);
    EXPECT_EQ(circleOne.getCenter().y, circleOneCenter.y);
    EXPECT_EQ(circleTwo.getRadius(), circleTwoRadius);
    EXPECT_EQ(circleTwo.getCenter().x, circleTwoCenter.x);
    EXPECT_EQ(circleTwo.getCenter().y, circleTwoCenter.y);
    EXPECT_EQ(circleThree.getRadius(), circleThreeRadius);
    EXPECT_EQ(circleThree.getCenter().x, circleThreeCenter.x);
    EXPECT_EQ(circleThree.getCenter().y, circleThreeCenter.y);
    EXPECT_EQ(circleFour.getRadius(), circleFourRadius);
    EXPECT_EQ(circleFour.getCenter().x, circleFourCenter.x);
    EXPECT_EQ(circleFour.getCenter().y, circleFourCenter.y);
}

TEST_F(CircleTest, SetRadius) {
    circleOne.setRadius(2.0);
    EXPECT_EQ(circleOne.getRadius(), 2.0);
}

TEST_F(CircleTest, SetCenter) {
    circleOne.setCenter(3.5, -4.5);
    EXPECT_EQ(circleOne.getCenter().x, 3.5);
    EXPECT_EQ(circleOne.getCenter().y, -4.5);
}

TEST_F(CircleTest, ScaleShape) {
    circleThree.scaleShape(2.0);
    EXPECT_EQ(circleThree.getRadius(), 2 * circleThreeRadius);
    EXPECT_EQ(circleThree.getCenter().x, circleThreeCenter.x);
    EXPECT_EQ(circleThree.getCenter().y, circleThreeCenter.y);
}

TEST_F(CircleTest, PrintParameters) { circleOne.printParameters(); }
