#pragma once

#include "commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class CausesBrakingIntersectionPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world1;
    std::shared_ptr<World> world2;
    CausesBrakingIntersectionPredicate pred;

  private:
    void SetUp() override;
};
