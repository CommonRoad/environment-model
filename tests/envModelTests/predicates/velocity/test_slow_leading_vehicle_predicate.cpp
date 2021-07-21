//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_slow_leading_vehicle_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void SlowLeadingVehiclePredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 2, 2, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateZeroObstacleFive = std::make_shared<State>(1, 0, 6, 60, 0, 0, 0, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 12, 2, 2, 0, 0, 0, 12, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 22, 2, 50, 0, 0, 0, 22, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 32, 2, 36, 0, 0, 0, 32, 0);
    std::shared_ptr<State> stateOneObstacleFive = std::make_shared<State>(1, 60, 9.5, 0, 0, 0, 0, 60, 1.5);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 14, 2, 2, 0, 0, 0, 14, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 72, 2, 12, 0, 0, 0, 72, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 68, 2, 36, 0, 0, 0, 68, 0);
    std::shared_ptr<State> stateTwoObstacleFour = std::make_shared<State>(2, 44, 2, 50, 0, 0, 0, 44, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 16, 2, 2, 0, 0, 0, 16, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 84, 2, 2, 0, 0, 0, 84, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 98, 2, 36, 0, 0, 0, 98, 0);
    std::shared_ptr<State> stateThreeObstacleFour = std::make_shared<State>(3, 94, 2, 0, 0, 0, 0, 94, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleFour{
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFour),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFour)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleFive{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFive),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFive)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, false, stateOneObstacleTwo, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree = std::make_shared<Obstacle>(Obstacle(3, false, stateOneObstacleThree, ObstacleType::car, 50, 10, 3,
                                                        -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    obstacleFour = std::make_shared<Obstacle>(Obstacle(4, false, stateTwoObstacleFour, ObstacleType::car, 50, 10, 3,
                                                       -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));
    obstacleFive = std::make_shared<Obstacle>(Obstacle(5, false, stateZeroObstacleFive, ObstacleType::car, 50, 10, 3,
                                                       -10, 0.3, trajectoryPredictionObstacleFive, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(
        World(0, roadNetwork, {obstacleOne}, {obstacleTwo, obstacleThree, obstacleFour, obstacleFive}));
}

TEST_F(SlowLeadingVehiclePredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // no leading vehicle at all
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne)); // two leading vehicles which drive with speed limit
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));  // first leading vehicle is drives to slow
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne));  // third leading vehicle drives to slow
}

TEST_F(SlowLeadingVehiclePredicateTest, StatisticBooleanEvaluation) {
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, obstacleOne)); // no leading vehicle at all
    EXPECT_EQ(pred.getStatistics().numExecutions, 1);
    EXPECT_FALSE(
        pred.statisticBooleanEvaluation(1, world, obstacleOne)); // two leading vehicles which drive with speed limit
    EXPECT_EQ(pred.getStatistics().numExecutions, 2);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(2, world, obstacleOne)); // first leading vehicle is drives to slow
    EXPECT_EQ(pred.getStatistics().numExecutions, 3);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(3, world, obstacleOne)); // third leading vehicle drives to slow
    EXPECT_EQ(pred.getStatistics().numExecutions, 4);
}
