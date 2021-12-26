//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include <gtest/gtest.h>

#include "commonroad_cpp/geometry/circle.h"

class CircleTest : public testing::Test {
  protected:
    Circle circleOne;
    double circleOneRadius{0.0};
    vertex circleOneCenter{0.0, 0.0};
    Circle circleTwo;
    double circleTwoRadius{1.0};
    vertex circleTwoCenter{0.0, 0.0};
    Circle circleThree;
    double circleThreeRadius{1.0};
    Circle circleFour;
    double circleFourRadius{-1.0};
    vertex circleFourCenter{-1.0, -1.0};

  private:
    void SetUp() override;
};
