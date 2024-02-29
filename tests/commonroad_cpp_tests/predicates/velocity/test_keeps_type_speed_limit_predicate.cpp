//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_keeps_type_speed_limit_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle.h"

void KeepsTypeSpeedLimitPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 2, 22.22, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 30.0, 0, 0);

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 30, 2, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 40, 2, 22.22, 0, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 50, 2, 30.0, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};
    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo)};

    obstacleOne =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::truck, 30, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleOne, 10, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, 0.1));
}

TEST_F(KeepsTypeSpeedLimitPredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne)); // truck drives slower than type speed limit
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne)); // truck drives exactly type speed limit
    EXPECT_FALSE(pred.booleanEvaluation(2, world,
                                        obstacleOne)); // truck drives faster than type speed limit
    EXPECT_TRUE(pred.booleanEvaluation(0, world,
                                       obstacleTwo)); // car drives slower than truck type speed limit
    EXPECT_TRUE(pred.booleanEvaluation(1, world,
                                       obstacleTwo)); // car drives slower than truck type speed limit
    EXPECT_TRUE(pred.booleanEvaluation(2, world,
                                       obstacleTwo)); // car drives slower than truck type speed limit
}

TEST_F(KeepsTypeSpeedLimitPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(KeepsTypeSpeedLimitPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
