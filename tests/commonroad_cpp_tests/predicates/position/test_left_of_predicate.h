#pragma once
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/left_of_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class LeftOfPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    LeftOfPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
