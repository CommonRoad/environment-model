//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_at_red_left_traffic_light_predicate.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"

void AtRedLeftTrafficLightPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TrafficLightTest-1_1_T-1.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 26.5, -15.0, 0, 0, M_PI / 2, 0, 15.0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 40.0, 2.5, 0, 0, 0, 0, 10, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 26.5, -3.5, 0, 0, M_PI / 2, 0, 20, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 30.0, 2.5, 0, 0, 0, 0, 20, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 26.5, 1.5, 0, 0, M_PI / 2, 0, 25.0, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 26.5, 15.0, 0, 0, M_PI / 2, 0, 38.5, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, false, stateZeroObstacleTwo, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, timeStepSize));
}

TEST_F(AtRedLeftTrafficLightPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(
        0, world, obstacleOne)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));  // standing on stop line -> partially in intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne)); // inside intersection
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne)); // left intersection
    EXPECT_FALSE(pred.booleanEvaluation(
        0, world,
        obstacleTwo)); // in front of intersection/traffic light with another direction -> completely on incoming
    EXPECT_FALSE(pred.booleanEvaluation(
        1, world,
        obstacleTwo)); // standing on stop line -> partially in intersection, traffic light has another direction
}

TEST_F(AtRedLeftTrafficLightPredicateTest, StatisticBooleanEvaluation) {
    EXPECT_TRUE(pred.statisticBooleanEvaluation(
        0, world, obstacleOne)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(1, world, obstacleOne)); // standing on stop line -> partially in intersection
    EXPECT_FALSE(pred.statisticBooleanEvaluation(2, world, obstacleOne)); // inside intersection
    EXPECT_FALSE(pred.statisticBooleanEvaluation(3, world, obstacleOne)); // left intersection
    EXPECT_FALSE(pred.statisticBooleanEvaluation(
        0, world,
        obstacleTwo)); // in front of intersection/traffic light with another direction -> completely on incoming
    EXPECT_FALSE(pred.statisticBooleanEvaluation(
        1, world,
        obstacleTwo)); // standing on stop line -> partially in intersection, traffic light has another direction
}

TEST_F(AtRedLeftTrafficLightPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(AtRedLeftTrafficLightPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}