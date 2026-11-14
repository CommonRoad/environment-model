#pragma once
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/velocity/drives_faster_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class DrivesFasterPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    DrivesFasterPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
