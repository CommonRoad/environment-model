#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/velocity/slow_other_vehicle_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class SlowOtherVehiclePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    std::shared_ptr<Obstacle> obstacleFive;
    SlowOtherVehiclePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
