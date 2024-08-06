//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_drives_with_slightly_higher_speed_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void DrivesWithSlightlyHigherSpeedPredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 5, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 0, 10, 0, 0, 0, 10, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 5, 0, 10, 0, 0, 0, 5, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 20, 0, 10, 0, 0, 0, 20, 0);

    /* State 2 */
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 15, 0, 15, 0, 0, 0, 15, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 30, 0, 10, 0, 0, 0, 30, 0);

    /* State 3 */
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 0, 20, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 40, 0, 10, 0, 0, 0, 40, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle)};

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {obstacleOne}, 0.1));
}

TEST_F(DrivesWithSlightlyHigherSpeedPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, obstacleOne));
}

TEST_F(DrivesWithSlightlyHigherSpeedPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle, obstacleOne), std::runtime_error);
}

TEST_F(DrivesWithSlightlyHigherSpeedPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle, obstacleOne), std::runtime_error);
}
