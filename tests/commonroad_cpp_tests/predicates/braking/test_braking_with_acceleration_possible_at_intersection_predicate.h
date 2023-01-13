//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/predicates/braking/braking_with_acceleration_possible_at_intersection_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class BrakingWithAccelerationPossibleAtIntersectionPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world1;
    std::shared_ptr<World> world2;
    BrakingWithAccelerationPossibleAtIntersectionPredicate pred;

  private:
    void SetUp() override;
};
