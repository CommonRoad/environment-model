//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class RegulatoryElementsUtilsTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
