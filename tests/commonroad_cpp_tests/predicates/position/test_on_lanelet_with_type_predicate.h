//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/on_lanelet_with_type_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class OnLaneletWithTypePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    OnLaneletWithTypePredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<OptionalPredicateParameters> opt;

    void initializeTestData(LaneletType laneletType1, LaneletType laneletType2);

  private:
    void SetUp() override;
};
