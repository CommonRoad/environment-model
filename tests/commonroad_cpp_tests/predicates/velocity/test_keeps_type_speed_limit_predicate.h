#pragma once

#include "commonroad_cpp/predicates/velocity/keeps_type_speed_limit_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class KeepsTypeSpeedLimitPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    KeepsTypeSpeedLimitPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
