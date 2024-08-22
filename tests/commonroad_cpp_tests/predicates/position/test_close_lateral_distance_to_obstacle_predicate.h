
#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/close_lateral_distance_to_obstacle_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class CloseLateralDistanceToObstaclePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    CloseLateralDistanceToObstaclePredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<World> world2;
    void setUpLeft();
    void setUpRight();
};
