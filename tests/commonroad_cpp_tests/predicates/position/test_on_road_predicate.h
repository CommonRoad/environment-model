//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once
#include "../../interfaces/utility_functions.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/on_road_predicate.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
#include <gtest/gtest.h>

class OnRoadPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    OnRoadPredicate pred;
    std::shared_ptr<World> world;

    void initializeTestData(LaneletType laneletTypeRight, LaneletType laneletTypeLeft,
                            LaneletType laneletTypeSuccessorRight, LaneletType laneletTypeSuccessorLeft);

  private:
    void SetUp() override;
};
