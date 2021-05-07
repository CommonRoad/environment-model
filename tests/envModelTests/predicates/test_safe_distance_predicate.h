//
// Created by sebastian on 26.12.20.
//

#ifndef ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H
#define ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H

#include "../interfaces/utility_functions.h"
#include "obstacle/obstacle.h"
#include "predicates/braking/safe_distance_predicate.h"
#include "roadNetwork/road_network.h"
#include "world.h"
#include <gtest/gtest.h>

class SafeDistancePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> otherVehicle;
    SafeDistancePredicate pred;

  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H
