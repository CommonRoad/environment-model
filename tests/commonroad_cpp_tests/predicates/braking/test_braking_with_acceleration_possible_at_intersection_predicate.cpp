//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_braking_with_acceleration_possible_at_intersection_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle.h"

void BrakingWithAccelerationPossibleAtIntersectionPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 30, 4.5, 0, -2, 0);
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 130, 2, 0, -2, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 140, 2, 100, -1, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 150, 2, 10, 1, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 160, 2, 20, 2, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 170, 2, 10, 2, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 180, 2, 10, 2, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(
        Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car, 50, 10, 3, -10, 0.3, {}, 5, 2));

    auto roadNetwork1{utils_predicate_test::create_road_network({LaneletType::incoming}, {LaneletType::intersection})};
    auto roadNetwork2{utils_predicate_test::create_road_network_2()};

    world1 = std::make_shared<World>(
        World("testWorld", 0, roadNetwork1, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
    world2 = std::make_shared<World>(
        World("testWorld", 0, roadNetwork2, std::vector<std::shared_ptr<Obstacle>>{obstacleTwo}, {}, 0.1));
}

TEST_F(BrakingWithAccelerationPossibleAtIntersectionPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world1, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(2, world1, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(4, world1, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(5, world1, obstacleOne));

    EXPECT_TRUE(pred.booleanEvaluation(0, world2, obstacleTwo));
}

TEST_F(BrakingWithAccelerationPossibleAtIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world1, obstacleOne), std::runtime_error);
}

TEST_F(BrakingWithAccelerationPossibleAtIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world1, obstacleOne), std::runtime_error);
}