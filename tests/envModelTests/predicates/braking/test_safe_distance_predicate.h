//
// Created by sebastian on 26.12.20.
//

#ifndef ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H
#define ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/braking/safe_distance_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class SafeDistancePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    SafeDistancePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H
