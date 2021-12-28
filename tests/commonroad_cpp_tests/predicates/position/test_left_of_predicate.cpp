//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_left_of_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void LeftOfPredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 2, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 5, 6, 5, 0, 0, 0, 5, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 2, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 6, 11, 0, 0, 0, 10, 4);

    /* State 2 */
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 21, 6, 8, 0, 0, 0, 21, 4);

    /* State 3 */
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 2, 10, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 29, 6, 8, 0, 0, 0, 29, 4);

    /* State 4 */
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 40, 2, 10, 0, 0, 0, 40, 0);
    std::shared_ptr<State> stateFourObstacleTwo = std::make_shared<State>(4, 40, 6, 10, 0, 0, 0, 40, 4);

    /* State 5 */
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 50, 2, 10, 0, 0, 0, 50, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 55, 6, 10, 0, 0, 0, 55, 0);

    /* State 6 */
    std::shared_ptr<State> stateSixEgoVehicle = std::make_shared<State>(6, 60, 2, 10, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateSixObstacleOne = std::make_shared<State>(6, 65, -2, 5, 0, 0, 0, 65, -4);

    /* State 7 */
    std::shared_ptr<State> stateSevenEgoVehicle = std::make_shared<State>(7, 70, 2, 10, 0, 0, 0, 70, 0);
    std::shared_ptr<State> stateSevenObstacleOne = std::make_shared<State>(7, 70, -2, 11, 0, 0, 0, 70, -4);

    /* State 8 */
    std::shared_ptr<State> stateEightEgoVehicle = std::make_shared<State>(8, 80, 2, 10, 0, 0, 0, 80, 0);
    std::shared_ptr<State> stateEightObstacleOne = std::make_shared<State>(8, 81, -2, 8, 0, 0, 0, 81, -4);

    /* State 9 */
    std::shared_ptr<State> stateNineEgoVehicle = std::make_shared<State>(9, 90, 2, 10, 0, 0, 0, 90, 0);
    std::shared_ptr<State> stateNineObstacleOne = std::make_shared<State>(9, 89, -2, 10, 0, 0, 0, 89, -4);

    /* State 10 */
    std::shared_ptr<State> stateTenEgoVehicle = std::make_shared<State>(10, 100, 2, 10, 0, 0, 0, 100, 0);
    std::shared_ptr<State> stateTenObstacleTwo = std::make_shared<State>(10, 100, -2, 10, 0, 0, 0, 100, -4);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(6, stateSixEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(8, stateEightEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(9, stateNineEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(10, stateTenEgoVehicle)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleOne),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleOne),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleOne),
        std::pair<int, std::shared_ptr<State>>(9, stateNineObstacleOne)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(10, stateTenObstacleTwo)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroEgoVehicle, ObstacleType::car, 50, 10, 3, -10,
                                                     0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(3, false, stateFourObstacleTwo, ObstacleType::truck, 50, 10, 3,
                                                      -10, 0.3, trajectoryPredictionObstacleTwo, 8, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {obstacleOne, obstacleTwo}, 0.1));
}

TEST_F(LeftOfPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(
        pred.booleanEvaluation(0, world, egoVehicle, obstacleOne)); // other vehicle in left lane but not adjacent
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, obstacleOne)); // other vehicle exactly left of
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, obstacleOne)); // other vehicle partially left of in front
    EXPECT_TRUE(pred.booleanEvaluation(3, world, egoVehicle, obstacleOne)); // other vehicle partially left of in behind
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle, obstacleTwo)); // other vehicle left of in front and behind
    EXPECT_FALSE(pred.booleanEvaluation(5, world, egoVehicle, obstacleOne)); // other vehicle in same lane in front
    EXPECT_FALSE(
        pred.booleanEvaluation(6, world, egoVehicle, obstacleOne)); // other vehicle in right lane but not adjacent
    EXPECT_FALSE(pred.booleanEvaluation(7, world, egoVehicle, obstacleOne)); // other vehicle exactly right of
    EXPECT_FALSE(
        pred.booleanEvaluation(8, world, egoVehicle, obstacleOne)); // other vehicle partially right of in front
    EXPECT_FALSE(pred.booleanEvaluation(9, world, egoVehicle, obstacleOne)); // other vehicle partially right of behind
    EXPECT_FALSE(pred.booleanEvaluation(10, world, egoVehicle,
                                        obstacleTwo)); // other vehicle partially right of in front and behind
}

TEST_F(LeftOfPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(LeftOfPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}