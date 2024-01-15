//
// Created by Sebastian Maierhof.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_obstacle_is_static_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <math.h>

void ObstacleIsStaticPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 0, 10, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 0, 10, 0, 0, 0, 30, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
    };

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::STATIC, stateZeroEgoVehicle, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, {obstacleOne}, {}, 0.1));
    world2 = std::make_shared<World>(World(0, roadNetwork, {obstacleTwo}, {}, 0.1));
}

TEST_F(ObstacleIsStaticPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne));

    EXPECT_TRUE(pred.booleanEvaluation(0, world2, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, world2, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(3, world2, obstacleTwo));
}

TEST_F(ObstacleIsStaticPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(ObstacleIsStaticPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
