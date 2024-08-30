#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/right_of_broad_lane_marking_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class RightOfBroadLaneMarkingPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    RightOfBroadLaneMarkingPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
