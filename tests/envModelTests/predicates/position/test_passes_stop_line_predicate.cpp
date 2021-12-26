//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_passes_stop_line_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"

void PassesStopLinePredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 17.5, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 0, 6, 17.5, 0, 0, 0, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 17.5, 2, 2.5, 0, 0, 0, 17.5, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 17.5, 6, 0.5, 0, 0, 0, 17.5, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 5.0, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 17.55, 6, 4.95, 0, 0, 0, 17.55, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 25, 2, 10.0, 0, 0, 0, 25, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 25, 6, 10.0, 0, 0, 0, 25, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));

    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, false, stateZeroObstacleTwo, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleTwo, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    obstacleOne->setOccupiedLanes(roadNetwork->getLanes(), 0);
    obstacleTwo->setOccupiedLanes(roadNetwork->getLanes(), 0);

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, 0.1));
}

TEST_F(PassesStopLinePredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // stop line completely in front
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne)); // stop line exactly at obstacle front
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));  // obstacle on stop line
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne)); // stop line behind obstacle
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleTwo)); // stop line completely in front
    EXPECT_FALSE(pred.booleanEvaluation(
        1, world, obstacleTwo)); // stop line exactly at obstacle front (one min lon. position of stop line)
    EXPECT_TRUE(pred.booleanEvaluation(
        2, world, obstacleTwo)); // stop line with two different lon. positions; obstacle above stop line
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleTwo)); // stop line behind obstacle
}

TEST_F(PassesStopLinePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(PassesStopLinePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}