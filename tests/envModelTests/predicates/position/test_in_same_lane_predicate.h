//
// Created by Sebastian Maierhofer on 23.05.21.
//

#ifndef ENV_MODEL_TESTS_ENVMODELTESTS_PREDICATES_POSITION_TEST_IN_SAME_LANE_PREDICATE_H_
#define ENV_MODEL_TESTS_ENVMODELTESTS_PREDICATES_POSITION_TEST_IN_SAME_LANE_PREDICATE_H_

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestInSameLanePredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    InSameLanePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TESTS_ENVMODELTESTS_PREDICATES_POSITION_TEST_IN_SAME_LANE_PREDICATE_H_
