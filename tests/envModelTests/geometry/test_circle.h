//
// Created by sebastian on 26.12.20.
//

#ifndef ENV_MODEL_TEST_CIRCLE_H
#define ENV_MODEL_TEST_CIRCLE_H

#include <gtest/gtest.h>

#include "commonroad_cpp/geometry/circle.h"

class CircleTestInitialization {
  protected:
    Circle circleOne;
    double circleOneRadius{0.0};
    vertex circleOneCenter{0.0, 0.0};
    Circle circleTwo;
    double circleTwoRadius{1.0};
    vertex circleTwoCenter{0.0, 0.0};
    Circle circleThree;
    double circleThreeRadius{1.0};
    vertex circleThreeCenter{1.0, 1.0};
    Circle circleFour;
    double circleFourRadius{-1.0};
    vertex circleFourCenter{-1.0, -1.0};

    void setUpCircle();
};

class CircleTest : public testing::Test, public CircleTestInitialization {
  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_CIRCLE_H
