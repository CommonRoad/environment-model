//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_exist_standing_leading_vehicle_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void ExistStandingLeadingVehiclePredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 2, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroObstacleFour = std::make_shared<State>(0, 0, 0, 60, 0, 0, 0, -10, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 2, 0, 2, 0, 0, 0, 2, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 12, 0, 2, 0, 0, 0, 12, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 22, 0, 2, 0, 0, 0, 22, 0);
    std::shared_ptr<State> stateOneObstacleFour = std::make_shared<State>(1, 50, 3.5, 0, 0, 0, 0, 50, 3.5);

    /* State 2 */
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 4, 0, 2, 0, 0, 0, 4, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 14, 0, 0, 0, 0, 0, 14, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 24, 0, 2, 0, 0, 0, 24, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 34, 0, 2, 0, 0, 0, 34, 0);

    /* State 3 */
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 6, 0, 2, 0, 0, 0, 6, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 14, 0, 2, 0, 0, 0, 14, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 26, 0, 2, 0, 0, 0, 26, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 36, 0, 0, 0, 0, 0, 36, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
    };

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleFour{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFour),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFour),
    };

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroEgoVehicle, ObstacleType::car, 50, 10, 3, -10,
                                                     0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, false, stateOneObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(3, false, stateOneObstacleTwo, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree = std::make_shared<Obstacle>(Obstacle(4, false, stateTwoObstacleThree, ObstacleType::car, 50, 10, 3,
                                                        -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    obstacleFour = std::make_shared<Obstacle>(Obstacle(5, false, stateZeroObstacleFour, ObstacleType::car, 50, 10, 3,
                                                       -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world0 = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {obstacleFour}, 0.1));

    world1 =
        std::make_shared<World>(World(1, roadNetwork, {egoVehicle}, {obstacleOne, obstacleTwo, obstacleFour}, 0.1));

    world2 =
        std::make_shared<World>(World(2, roadNetwork, {egoVehicle}, {obstacleOne, obstacleTwo, obstacleThree}, 0.1));

    world3 =
        std::make_shared<World>(World(3, roadNetwork, {egoVehicle}, {obstacleOne, obstacleTwo, obstacleThree}, 0.1));
}

TEST_F(ExistStandingLeadingVehiclePredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world0, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, world1, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(3, world3, egoVehicle));
}