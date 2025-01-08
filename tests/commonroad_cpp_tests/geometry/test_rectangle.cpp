#include <cmath>

#include "test_rectangle.h"

void RectangleTestInitialization::SetUpRectangle() {
    rectangleOne = Rectangle();
    rectangleTwo = Rectangle(rectangleTwoLength, rectangleTwoWidth);
    rectangleThree = Rectangle(rectangleThreeLength, rectangleThreeWidth);
}

void RectangleTest::SetUp() { SetUpRectangle(); }

TEST_F(RectangleTest, Initialization) {
    EXPECT_EQ(rectangleOne.getType(), ShapeType::rectangle);
    EXPECT_EQ(rectangleOne.getLength(), rectangleOneLength);
    EXPECT_EQ(rectangleOne.getWidth(), rectangleOneWidth);
    EXPECT_EQ(rectangleTwo.getLength(), rectangleTwoLength);
    EXPECT_EQ(rectangleTwo.getWidth(), rectangleTwoWidth);
    EXPECT_EQ(rectangleThree.getLength(), rectangleThreeLength);
    EXPECT_EQ(rectangleThree.getWidth(), rectangleThreeWidth);
}

TEST_F(RectangleTest, SetWidth) {
    rectangleOne.setWidth(2.1);
    EXPECT_EQ(rectangleOne.getWidth(), 2.1);
}

TEST_F(RectangleTest, SetLength) {
    rectangleOne.setLength(2.1);
    EXPECT_EQ(rectangleOne.getLength(), 2.1);
}

TEST_F(RectangleTest, ScaleShape) {
    rectangleTwo.scaleShape(2.0);
    EXPECT_EQ(rectangleTwo.getWidth(), 2 * rectangleTwoWidth);
    EXPECT_EQ(rectangleTwo.getLength(), 2 * rectangleTwoLength);
}

TEST_F(RectangleTest, PrintParameters) { rectangleOne.printParameters(); }

TEST_F(RectangleTest, GetCircumradius) {
    EXPECT_EQ(rectangleOne.getCircumradius(), std::hypot(rectangleOneLength, rectangleOneWidth) / 2.0);
    EXPECT_EQ(rectangleTwo.getCircumradius(), std::hypot(rectangleTwoLength, rectangleTwoWidth) / 2.0);
    EXPECT_EQ(rectangleThree.getCircumradius(), std::hypot(rectangleThreeLength, rectangleThreeWidth) / 2.0);
}
