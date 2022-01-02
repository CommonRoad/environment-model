//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/predicates/velocity/keeps_fov_speed_limit_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class KeepsFOVSpeedLimitPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    KeepsFOVSpeedLimitPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
