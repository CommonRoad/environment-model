#pragma once

#include "commonroad_cpp/predicates/velocity/keeps_fov_speed_limit_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class KeepsFovSpeedLimitPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    KeepsFovSpeedLimitPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
