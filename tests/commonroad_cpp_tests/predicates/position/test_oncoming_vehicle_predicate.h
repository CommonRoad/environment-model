#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/oncoming_vehicle_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestOncomingVehiclePredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleEgo;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    OncomingVehiclePredicate pred;
    std::string pathToTestFile;
    std::string pathToTestFileOncoming;
    void initObstacles();

  private:
    void SetUp() override;
};
