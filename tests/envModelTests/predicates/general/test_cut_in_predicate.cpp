//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_cut_in_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <cmath>

void CutInPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 2, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 0, 2, 10, 0, 0, 0, 0, 4);
    // std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 20, 2, 10, 1, 0, 0, 20, 0);

    std::shared_ptr<State> stateOneObstacleOne =
        std::make_shared<State>(1, 20, 4, 10, 0, (1.0 / 4.0) * M_PI, (1.0 / 4.0) * M_PI, 20, 2);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 4, 10, 0, 0, 0, 10, 4);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 0, 12, 10, 0, 0, 0, 0, 10);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 30, 6, 10, 0, 0, 0, 30, 4);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 4);

    std::shared_ptr<State> stateThreeObstacleOne =
        std::make_shared<State>(3, 40, 4, 10, 0, -(1.0 / 4.0) * M_PI, -(1.0 / 4.0) * M_PI, 40, 2);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 30, 2, 10, -0, 0, 0, 30, 4);

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

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, false, stateZeroObstacleTwo, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree = std::make_shared<Obstacle>(Obstacle(3, false, stateOneObstacleThree, ObstacleType::car, 50, 10, 3,
                                                        -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                          std::vector<std::shared_ptr<Obstacle>>{obstacleTwo, obstacleThree}));
}

TEST_F(CutInPredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne,
                                        obstacleTwo)); // before cut-in -> ego vehicle occupies only single lane
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo));  // during cut-in
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo)); // after cut-in
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, obstacleTwo)); // driving back to initial lane
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne,
                                        obstacleThree)); // during cut-in -> but other vehicles is in another lane
}

TEST_F(CutInPredicateTest, StatisticBooleanEvaluation) {
    EXPECT_FALSE(pred.statisticBooleanEvaluation(
        0, world, obstacleOne, obstacleTwo)); // before cut-in -> ego vehicle occupies only single lane
    EXPECT_TRUE(pred.statisticBooleanEvaluation(1, world, obstacleOne, obstacleTwo));  // during cut-in
    EXPECT_FALSE(pred.statisticBooleanEvaluation(2, world, obstacleOne, obstacleTwo)); // after cut-in
    EXPECT_FALSE(pred.statisticBooleanEvaluation(3, world, obstacleOne, obstacleTwo)); // driving back to initial lane
    EXPECT_FALSE(pred.statisticBooleanEvaluation(
        1, world, obstacleOne, obstacleThree)); // during cut-in -> but other vehicles is in another lane
}
