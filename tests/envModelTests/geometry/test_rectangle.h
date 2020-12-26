//
// Created by sebastian on 26.12.20.
//

#ifndef ENV_MODEL_TEST_RECTANGLE_H
#define ENV_MODEL_TEST_RECTANGLE_H

#include <gtest/gtest.h>
#include "geometry/rectangle.h"

class RectangleTestInitialization {
protected:
    Rectangle rectangleOne;
    double rectangleOneWidth { 1.8 };
    double rectangleOneLength { 4.5 };
    Rectangle rectangleTwo;
    double rectangleTwoWidth { 1.0 };
    double rectangleTwoLength { 4.0 };
    Rectangle rectangleThree;
    double rectangleThreeWidth { -1.0 };
    double rectangleThreeLength { -4.0 };

    void SetUpRectangle();
};

class RectangleTest : public testing::Test, public RectangleTestInitialization {
private:
    void SetUp() override;
};


#endif //ENV_MODEL_TEST_RECTANGLE_H
