//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/main_carriageway_right_lane_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class MainCarriagewayRightLanePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    MainCarriagewayRightLanePredicate pred;
    std::shared_ptr<World> worldOne;
    std::shared_ptr<World> worldTwo;
    std::shared_ptr<World> worldThree;

  private:
    void SetUp() override;
};
