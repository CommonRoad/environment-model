//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_drives_faster_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void DrivesFasterPredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 5, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 0, 10, 0, 0, 0, 10, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 5, 0, 20, 0, 0, 0, 5, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 20, 0, 20, 0, 0, 0, 20, 0);

    /* State 2 */
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 25, 0, 35, 0, 0, 0, 25, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 40, 0, 30, 0, 0, 0, 40, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
    };

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroEgoVehicle, ObstacleType::car, 50, 10, 3, -10,
                                                     0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, false, stateOneObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {obstacleOne}, 0.1));
}

TEST_F(DrivesFasterPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, obstacleOne));
}