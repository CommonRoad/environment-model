#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/velocity/keeps_min_speed_limit_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class KeepsMinSpeedLimitPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    KeepsMinSpeedLimitPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
