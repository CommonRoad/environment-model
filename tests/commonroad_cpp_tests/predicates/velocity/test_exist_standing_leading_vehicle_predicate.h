//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/velocity/exist_standing_leading_vehicle_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class ExistStandingLeadingVehiclePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    ExistStandingLeadingVehiclePredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<World> world0;
    std::shared_ptr<World> world1;
    std::shared_ptr<World> world2;
    std::shared_ptr<World> world3;

  private:
    void SetUp() override;
};
