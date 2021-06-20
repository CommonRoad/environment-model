//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_keeps_lane_speed_limit_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void KeepsLaneSpeedLimitPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 45, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 45, 2, 50, 0, 0, 0, 45, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(2, 95, 2, 55, 0, 0, 0, 95, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(3, 150, 2, 45, 0, 0, 0, 150, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateOneObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                          std::vector<std::shared_ptr<Obstacle>>{obstacleTwo}));
}

TEST_F(KeepsLaneSpeedLimitPredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne));  // vehicle drives with lower velocity
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));  // vehicle drives exactly with the max speed
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne)); // vehicle drives too fast
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne));  // there exists no speed limit
}

TEST_F(KeepsLaneSpeedLimitPredicateTest, StatisticBooleanEvaluation) {
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, obstacleOne)); // vehicle drives with lower velocity
    EXPECT_EQ(pred.getStatistics().numExecutions, 1);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(1, world, obstacleOne)); // vehicle drives exactly with the max speed
    EXPECT_EQ(pred.getStatistics().numExecutions, 2);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(2, world, obstacleOne)); // vehicle drives too fast
    EXPECT_EQ(pred.getStatistics().numExecutions, 3);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(3, world, obstacleOne)); // there exists no speed limit
    EXPECT_EQ(pred.getStatistics().numExecutions, 4);
}
