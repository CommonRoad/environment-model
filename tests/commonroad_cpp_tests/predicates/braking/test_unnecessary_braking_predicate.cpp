//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_unnecessary_braking_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void UnnecessaryBrakingPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, 1, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 10, 2, 10, 1, 0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 20, 2, 10, 1, 0);

    std::shared_ptr<State> stateOneObstacleOne{std::make_shared<State>()};
    stateOneObstacleOne->setTimeStep(1);
    stateOneObstacleOne->setXPosition(10);
    stateOneObstacleOne->setYPosition(2);
    stateOneObstacleOne->setVelocity(9.5);
    stateOneObstacleOne->setGlobalOrientation(0);
    stateOneObstacleOne->setLonPosition(10);
    stateOneObstacleOne->setLatPosition(0);
    stateOneObstacleOne->setCurvilinearOrientation(0);

    std::shared_ptr<State> stateOneObstacleTwo{std::make_shared<State>()};
    stateOneObstacleTwo->setTimeStep(1);
    stateOneObstacleTwo->setXPosition(20);
    stateOneObstacleTwo->setYPosition(2);
    stateOneObstacleTwo->setVelocity(9.6);
    stateOneObstacleTwo->setGlobalOrientation(0);
    stateOneObstacleTwo->setLonPosition(20);
    stateOneObstacleTwo->setLatPosition(0);
    stateOneObstacleTwo->setCurvilinearOrientation(0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 30, 2, 10, -1, 0, 0, 30, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 10, -7, 0, 0, 20, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 30, 2, 10, -2, 0, 0, 30, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 40, 2, 10, -3, 0, 0, 40, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 2, 10, -3, 0, 0, 30, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 40, 2, 10, -1.5, 0, 0, 40, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 50, 2, 10, -2, 0, 0, 50, 0);

    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 40, 2, 10, -8, 0, 0, 40, 0);

    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 50, 2, 10, 2, 0, 0, 50, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, false, stateZeroObstacleTwo, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree = std::make_shared<Obstacle>(Obstacle(3, false, stateZeroObstacleThree, ObstacleType::car, 50, 10, 3,
                                                        -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                          std::vector<std::shared_ptr<Obstacle>>{obstacleTwo, obstacleThree}, 0.1));
}

TEST_F(UnnecessaryBrakingPredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // a_ego > 0
    EXPECT_TRUE(
        pred.booleanEvaluation(1, world, obstacleOne)); // a_ego < a_lead - |a_abrupt| for single leading vehicle
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));  // a_ego < a_lead - |a_abrupt| for all leading vehicles
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne)); // a_ego > a_lead - |a_abrupt| for all leading vehicles
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleOne));  // a_ego < a_abrupt; no leading vehicle
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleOne)); // a_ego > a_abrupt; no leading vehicle
}

TEST_F(UnnecessaryBrakingPredicateTest, ConstraintEvaluation) {
    EXPECT_NEAR(pred.constraintEvaluation(0, world, obstacleOne).realValuedConstraint, -1, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(1, world, obstacleOne).realValuedConstraint, -3, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(2, world, obstacleOne).realValuedConstraint, -4, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(3, world, obstacleOne).realValuedConstraint, -3.5, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(4, world, obstacleOne).realValuedConstraint, -2, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(5, world, obstacleOne).realValuedConstraint, -2, 0.001);
}

TEST_F(UnnecessaryBrakingPredicateTest, RobustEvaluation) {
    EXPECT_NEAR(pred.robustEvaluation(0, world, obstacleOne), -2, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(1, world, obstacleOne), 2, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(2, world, obstacleOne), 3, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(3, world, obstacleOne), -0.5, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(4, world, obstacleOne), 6, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(5, world, obstacleOne), -4, 0.001);
}

TEST_F(UnnecessaryBrakingPredicateTest, StatisticBooleanEvaluation) {
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, obstacleOne));
    EXPECT_EQ(pred.getStatistics().numExecutions, 1);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(1, world, obstacleOne));
    EXPECT_EQ(pred.getStatistics().numExecutions, 2);
}
