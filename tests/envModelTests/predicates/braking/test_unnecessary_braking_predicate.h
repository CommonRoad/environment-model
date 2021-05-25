//
// Created by sebastian on 26.12.20.
//

#ifndef ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H
#define ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/braking/unnecessary_braking_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class UnnecessaryBrakingPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<World> world;
    UnnecessaryBrakingPredicate pred;

  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_SAFE_DISTANCE_PREDICATE_H
