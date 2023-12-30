//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_keeps_fov_speed_limit_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle.h"

void KeepsFovSpeedLimitPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 25, 2, 25, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 50, 2, 50, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 55, 2, 55, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    obstacleOne =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::truck, 30, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleOne, 10, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world =
        std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
}

TEST_F(KeepsFovSpeedLimitPredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne)); // obstacle stands
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne)); // obstacle drives below speed limit
    EXPECT_TRUE(pred.booleanEvaluation(2, world,
                                       obstacleOne)); // obstacle drives exactly speed limit
    EXPECT_FALSE(pred.booleanEvaluation(3, world,
                                        obstacleOne)); // obstacle drives above speed limit
}

TEST_F(KeepsFovSpeedLimitPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(KeepsFovSpeedLimitPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
