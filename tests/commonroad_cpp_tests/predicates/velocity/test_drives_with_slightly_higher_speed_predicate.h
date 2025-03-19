#pragma once
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/velocity/drives_with_slightly_higher_speed_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class DrivesWithSlightlyHigherSpeedPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    DrivesWithSlightlyHigherSpeedPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
