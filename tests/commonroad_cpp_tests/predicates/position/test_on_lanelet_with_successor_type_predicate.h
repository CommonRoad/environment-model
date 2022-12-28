//
// Created by valentin on 04.11.22.
//

#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/on_lanelet_with_successor_type_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class OnLaneletWithSuccessorTypePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    OnLaneletWithSuccessorTypePredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<OptionalPredicateParameters> opt;

    void initializeTestData(LaneletType laneletTypeRight, LaneletType laneletTypeLeft,
                            LaneletType laneletTypeSuccessorRight, LaneletType laneletTypeSuccessorLeft);

  private:
    void SetUp() override;
};
