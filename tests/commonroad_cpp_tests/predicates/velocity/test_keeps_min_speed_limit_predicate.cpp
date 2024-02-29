//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_keeps_min_speed_limit_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void KeepsMinSpeedLimitPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 5, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 5, 2, 10, 0, 0, 0, 5, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 15, 2, 15, 0, 0, 0, 15, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 6, 5, 0, 0, 0, 30, 4);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
}

TEST_F(KeepsMinSpeedLimitPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // vehicle drives too slow
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));  // vehicle drives exactly with the min. required speed
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));  // vehicle drives with higher velocity
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne));  // there exists not a min. required speed
}

TEST_F(KeepsMinSpeedLimitPredicateTest, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat)); // vehicle drives too slow
    EXPECT_EQ(stat->numExecutions, 1);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(1, world, obstacleOne, timer,
                                                stat)); // vehicle drives exactly with the min. required speed
    EXPECT_EQ(stat->numExecutions, 2);
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(2, world, obstacleOne, timer, stat)); // vehicle drives with higher velocity
    EXPECT_EQ(stat->numExecutions, 3);
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(3, world, obstacleOne, timer, stat)); // there exists not a min. required speed
    EXPECT_EQ(stat->numExecutions, 4);
}

TEST_F(KeepsMinSpeedLimitPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(KeepsMinSpeedLimitPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
