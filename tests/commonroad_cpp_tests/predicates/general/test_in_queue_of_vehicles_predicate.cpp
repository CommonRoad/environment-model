//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_in_queue_of_vehicles_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void InQueueOfVehiclesPredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 2, 0, 0, 0, 0, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 15, 0, 2, 0, 0, 0, 15, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 30, 0, 2, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 45, 0, 2, 0, 0, 0, 45, 0);

    /* State 2 */
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 30, 0, 2, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 45, 0, 2, 0, 0, 0, 45, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 60, 0, 2, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 75, 0, 10, 0, 0, 0, 75, 0);

    /* State 3 */
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 45, 0, 2, 0, 0, 0, 45, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 60, 0, 2, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 75, 0, 2, 0, 0, 0, 75, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 80, 0, 2, 0, 0, 0, 80, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
    };

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateOneObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateOneObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, stateTwoObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world =
        std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {obstacleOne, obstacleTwo, obstacleThree}, 0.1));
}

TEST_F(InQueueOfVehiclesPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, egoVehicle));
}

TEST_F(InQueueOfVehiclesPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(InQueueOfVehiclesPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}