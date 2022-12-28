#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/close_to_intersection_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class CloseToIntersectionPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    CloseToIntersectionPredicate pred;
    std::shared_ptr<World> world;

    void initializeTestData(LaneletType laneletTypeRight, LaneletType laneletTypeLeft,
                            LaneletType laneletTypeSuccessorRight, LaneletType laneletTypeSuccessorLeft);

  private:
    void SetUp() override;
};
