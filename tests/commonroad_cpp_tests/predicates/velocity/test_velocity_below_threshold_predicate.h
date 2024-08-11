#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/velocity/velocity_below_threshold_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class VelocityBelowThresholdPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    VelocityBelowThresholdPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
