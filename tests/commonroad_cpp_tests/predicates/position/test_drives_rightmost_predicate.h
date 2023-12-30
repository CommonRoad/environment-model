//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/drives_rightmost_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class DrivesRightmostPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    DrivesRightmostPredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<World> world2;

  private:
    void SetUp() override;
};
