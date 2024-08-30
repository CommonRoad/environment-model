#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/intersection/at_same_intersection_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class AtSameIntersectionPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<World> world;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    AtSameIntersectionPredicate pred;

  private:
    void SetUp() override;
};
