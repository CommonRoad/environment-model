//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_in_slow_moving_traffic_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void InSlowMovingTrafficPredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 0, 6, 0, 0, 0, 0, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 6, 0, 6, 0, 0, 0, 6, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 16, 0, 6, 0, 0, 0, 16, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 26, 0, 6, 0, 0, 0, 22, 0);

    /* State 2 */
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 12, 0, 6, 0, 0, 0, 12, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 22, 0, 6, 0, 0, 0, 22, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 32, 0, 6, 0, 0, 0, 24, 0);
    std::shared_ptr<State> stateTwoObstacleFour = std::make_shared<State>(2, 42, 0, 12, 0, 0, 0, 34, 0);

    /* State 3 */
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 18, 0, 6, 0, 0, 0, 18, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 28, 0, 6, 0, 0, 0, 28, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 38, 0, 6, 0, 0, 0, 26, 0);
    std::shared_ptr<State> stateThreeObstacleFour = std::make_shared<State>(3, 48, 0, 6, 0, 0, 0, 36, 0);

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

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, false, stateOneObstacleTwo, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree = std::make_shared<Obstacle>(Obstacle(3, false, stateOneObstacleThree, ObstacleType::car, 50, 10, 3,
                                                        -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    obstacleFour = std::make_shared<Obstacle>(Obstacle(4, false, stateTwoObstacleFour, ObstacleType::car, 50, 10, 3,
                                                       -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world =
        std::make_shared<World>(World(0, roadNetwork, {obstacleOne}, {obstacleTwo, obstacleThree, obstacleFour}, 0.1));
}

TEST_F(InSlowMovingTrafficPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne));
}