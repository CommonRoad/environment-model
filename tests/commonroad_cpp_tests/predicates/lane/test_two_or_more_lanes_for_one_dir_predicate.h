#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/two_or_more_lanes_for_one_dir_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TwoOrMoreLanesPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<World> world;
    std::shared_ptr<Obstacle> egoVehicle_2;
    std::shared_ptr<World> world_2;
    TwoOrMoreLanesForOneDirPredicate pred;

  private:
    void SetUp() override;
};
