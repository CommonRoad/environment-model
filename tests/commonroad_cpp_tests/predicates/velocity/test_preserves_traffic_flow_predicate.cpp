//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_preserves_traffic_flow_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void PreservesTrafficFlowPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 2, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 2, 2, 20, 0, 0, 0, 2, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 22, 2, 50, 0, 0, 0, 22, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world =
        std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
}

TEST_F(PreservesTrafficFlowPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // vehicle drives too slow
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne)); // vehicle drives at lower limit to
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));  // vehicle drives faster than velocity limit
}

TEST_F(PreservesTrafficFlowPredicateTest, StatisticBooleanEvaluation) {
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, obstacleOne)); // vehicle drives too slow
    EXPECT_EQ(pred.getStatistics().numExecutions, 1);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(1, world, obstacleOne)); // vehicle drives at lower limit to
    EXPECT_EQ(pred.getStatistics().numExecutions, 2);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(2, world, obstacleOne)); // vehicle drives faster than velocity limit
    EXPECT_EQ(pred.getStatistics().numExecutions, 3);
}

TEST_F(PreservesTrafficFlowPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(PreservesTrafficFlowPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
