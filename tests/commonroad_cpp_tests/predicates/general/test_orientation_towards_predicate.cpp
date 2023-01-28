//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_orientation_towards_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <cmath>

void OrientationTowardsPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 2, 10, 0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 0, 2, 10, 0, 0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 0, 12, 10, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 20, 4, 10, 0, (1.0 / 5.0) * M_PI);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 2, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 10, 12, 10, 0, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 30, 6, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 20, 2, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 20, 12, 10, 0, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 40, 2, 10, 0, -(1.0 / 5.0) * M_PI);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 30, 4, 10, -0, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 30, 12, 10, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network_3()};

    world = std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                          std::vector<std::shared_ptr<Obstacle>>{obstacleTwo, obstacleThree}, 0.1));
}

TEST_F(OrientationTowardsPredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne,
                                        obstacleTwo));                        // obs1 vehicle drives straight
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo));  // obs1 vehicle drives to other from left
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo)); // both drive straight
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, obstacleTwo));  // obs1 vehicle drives to other from right
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne,
                                       obstacleThree)); // obs1 drives to other from right
}

TEST_F(OrientationTowardsPredicateTest, StatisticBooleanEvaluation) {
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, obstacleOne,
                                                 obstacleTwo)); // obs1 vehicle drives straight
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(1, world, obstacleOne, obstacleTwo)); // obs1 vehicle drives to other from left
    EXPECT_FALSE(pred.statisticBooleanEvaluation(2, world, obstacleOne, obstacleTwo)); // both drive straight
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(3, world, obstacleOne, obstacleTwo)); // obs1 vehicle drives to other from right
    EXPECT_TRUE(pred.statisticBooleanEvaluation(3, world, obstacleOne,
                                                obstacleThree)); // obs1 drives to other from right
}

TEST_F(OrientationTowardsPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(OrientationTowardsPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}
