#pragma once

#include "commonroad_cpp/geometry/rectangle.h"
#include <gtest/gtest.h>

class RectangleTestInitialization {
  protected:
    Rectangle rectangleOne;
    double rectangleOneWidth{1.8};
    double rectangleOneLength{4.5};
    Rectangle rectangleTwo;
    double rectangleTwoWidth{1.0};
    double rectangleTwoLength{4.0};
    Rectangle rectangleThree;
    double rectangleThreeWidth{-1.0};
    double rectangleThreeLength{-4.0};

    void SetUpRectangle();
};

class RectangleTest : public testing::Test, public RectangleTestInitialization {
    void SetUp() override;
};
