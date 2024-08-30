#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/intersection/at_four_way_stop_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class AtFourWayStopPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<World> world;
    std::shared_ptr<World> world_2;
    std::shared_ptr<Obstacle> egoVehicle;
    AtFourWayStopPredicate pred;

  private:
    void SetUp() override;
};
