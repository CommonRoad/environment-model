#pragma once

#include "commonroad_cpp/predicates/braking/brakes_stronger_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class BrakesStrongerPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world;
    BrakesStrongerPredicate pred;

  private:
    void SetUp() override;
};
