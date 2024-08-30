#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/predicates/braking/braking_at_intersection_possible_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class BrakingWithAccelerationPossibleAtIntersectionPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world1;
    std::shared_ptr<World> world2;
    BrakingAtIntersectionPossiblePredicate pred;

  private:
    void SetUp() override;
};
