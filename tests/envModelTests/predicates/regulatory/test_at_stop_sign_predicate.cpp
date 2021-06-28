//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_at_stop_sign_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"

void AtStopSignPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, 0, 0, 0, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 4, 10, 0, 0, 0, 10, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 6, 10, 0, 0, 0, 20, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}));
}

TEST_F(AtStopSignPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne)); // occupied lanelet references stop sign
    EXPECT_TRUE(
        pred.booleanEvaluation(1, world, obstacleOne)); // one occupied lanelet references stop sign the other not
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne)); // occupied lanelet does not occupy stop sign
}
