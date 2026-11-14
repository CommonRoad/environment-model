#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/lane_based_orientation_side_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class LaneBasedOrientationSidePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<World> world;
    LaneBasedOrientationSidePredicate pred;

  private:
    void SetUp() override;
};
