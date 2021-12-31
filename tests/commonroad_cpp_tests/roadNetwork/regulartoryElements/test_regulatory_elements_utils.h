//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include <gtest/gtest.h>

class RegulatoryElementsUtilsTest : public testing::Test {
    std::shared_ptr<Obstacle> obstacleOne;

  private:
    void SetUp() override;
};
