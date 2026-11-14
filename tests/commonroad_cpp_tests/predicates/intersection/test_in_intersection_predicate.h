#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/intersection/in_intersection_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class InIntersectionPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<World> world;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    InIntersectionPredicate pred;

  private:
    void SetUp() override;
};
