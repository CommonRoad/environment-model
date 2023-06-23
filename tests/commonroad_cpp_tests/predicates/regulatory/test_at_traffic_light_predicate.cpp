//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_at_traffic_light_predicate.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void AtRedTrafficLightPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TrafficLight-1"};
    const auto &[obs, rn, ts] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    obstacles = obs;
    roadNetwork = rn;
    timeStepSize = ts;
}

TEST_F(AtRedTrafficLightPredicateTest, BooleanEvaluationDirectionAll) {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 23.5, 15.0, 0, 0, -M_PI / 2, 0, 11.5, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 40.0, 3.5, 0, 0, -M_PI, 0, 10, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 23.5, 6.5, 0, 0, -M_PI / 2, 0, 20, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 30.0, 3.5, 0, 0, -M_PI, 0, 20, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 23.5, 1.5, 0, 0, -M_PI / 2, 0, 25.0, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 23.5, -15.0, 0, 0, -M_PI / 2, 0, 41.5, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, timeStepSize));

    auto opt{std::make_shared<OptionalPredicateParameters>()};
    opt->trafficLightState = {TrafficLightState::red};
    opt->turningDirection = {TurningDirection::all};
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {},
                                       opt)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_TRUE(
        pred.booleanEvaluation(1, world, obstacleOne, {}, opt)); // standing on stop line -> partially in intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, {}, opt)); // inside intersection
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, {}, opt)); // left intersection
    EXPECT_TRUE(pred.booleanEvaluation(
        0, world, obstacleTwo, {},
        opt)); // in front of intersection/traffic light with another direction -> completely on incoming
    EXPECT_TRUE(pred.booleanEvaluation(
        1, world, obstacleTwo, {},
        opt)); // standing on stop line -> partially in intersection, traffic light has another direction
}

TEST_F(AtRedTrafficLightPredicateTest, StatisticBooleanEvaluationDirectionAll) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TrafficLight-1"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 23.5, 15.0, 0, 0, -M_PI / 2, 0, 11.5, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 40.0, 3.5, 0, 0, -M_PI, 0, 10, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 23.5, 6.5, 0, 0, -M_PI / 2, 0, 20, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 30.0, 3.5, 0, 0, -M_PI, 0, 20, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 23.5, 1.5, 0, 0, -M_PI / 2, 0, 25.0, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 23.5, -15.0, 0, 0, -M_PI / 2, 0, 41.5, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, timeStepSize));

    auto opt{std::make_shared<OptionalPredicateParameters>()};
    opt->trafficLightState = {TrafficLightState::red};
    opt->turningDirection = {TurningDirection::all};
    EXPECT_TRUE(pred.statisticBooleanEvaluation(
        0, world, obstacleOne, {}, opt)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_TRUE(pred.statisticBooleanEvaluation(1, world, obstacleOne, {},
                                                opt)); // standing on stop line -> partially in intersection
    EXPECT_FALSE(pred.statisticBooleanEvaluation(2, world, obstacleOne, {}, opt)); // inside intersection
    EXPECT_FALSE(pred.statisticBooleanEvaluation(3, world, obstacleOne, {}, opt)); // left intersection
    EXPECT_TRUE(pred.statisticBooleanEvaluation(
        0, world, obstacleTwo, {},
        opt)); // in front of intersection/traffic light with another direction -> completely on incoming
    EXPECT_TRUE(pred.statisticBooleanEvaluation(
        1, world, obstacleTwo, {},
        opt)); // standing on stop line -> partially in intersection, traffic light has another direction
}

TEST_F(AtRedTrafficLightPredicateTest, StatisticBooleanEvaluationDirectionRight) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TrafficLight-1"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 15.0, 0.0, 0, 0, 0, 0, 15.0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 26.5, -13.5, 0, 0, M_PI / 2, 0, 10, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 20.0, 0.0, 0, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 26.5, -3.5, 0, 0, M_PI / 2, 0, 20, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 25.0, 0.0, 0, 0, 0, 0, 25.0, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 40.0, 0.0, 0, 0, 0, 0, 40.0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, timeStepSize));

    auto opt{std::make_shared<OptionalPredicateParameters>()};
    opt->trafficLightState = {TrafficLightState::red};
    opt->turningDirection = {TurningDirection::right};
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {},
                                       opt)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_TRUE(
        pred.booleanEvaluation(1, world, obstacleOne, {}, opt)); // standing on stop line -> partially in intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, {}, opt)); // inside intersection
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, {}, opt)); // left intersection
    EXPECT_FALSE(pred.booleanEvaluation(
        0, world, obstacleTwo, {},
        opt)); // in front of intersection/traffic light with another direction -> completely on incoming
    EXPECT_FALSE(pred.booleanEvaluation(
        1, world, obstacleTwo, {},
        opt)); // standing on stop line -> partially in intersection, traffic light has another direction
}

TEST_F(AtRedTrafficLightPredicateTest, StatisticBooleanEvaluationDirectionLeft) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TrafficLight-1"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 26.5, -15.0, 0, 0, M_PI / 2, 0, 15.0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 40.0, 2.5, 0, 0, 0, 0, 10, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 26.5, -3.5, 0, 0, M_PI / 2, 0, 20, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 30.0, 2.5, 0, 0, 0, 0, 20, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 26.5, 1.5, 0, 0, M_PI / 2, 0, 25.0, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 26.5, 15.0, 0, 0, M_PI / 2, 0, 38.5, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, timeStepSize));

    auto opt{std::make_shared<OptionalPredicateParameters>()};
    opt->trafficLightState = {TrafficLightState::red};
    opt->turningDirection = {TurningDirection::left};
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {},
                                       opt)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_TRUE(
        pred.booleanEvaluation(1, world, obstacleOne, {}, opt)); // standing on stop line -> partially in intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, {}, opt)); // inside intersection
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, {}, opt)); // left intersection
    EXPECT_FALSE(pred.booleanEvaluation(
        0, world, obstacleTwo, {},
        opt)); // in front of intersection/traffic light with another direction -> completely on incoming
    EXPECT_FALSE(pred.booleanEvaluation(
        1, world, obstacleTwo, {},
        opt)); // standing on stop line -> partially in intersection, traffic light has another direction
}

TEST_F(AtRedTrafficLightPredicateTest, StatisticBooleanEvaluationDirectionStraight) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TrafficLight-1"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 40.0, 3.0, 0, 0, M_PI, 0, 10.0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 10.0, 0.0, 0, 0, 0, 0, 10, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 30.0, 3.0, 0, 0, M_PI, 0, 20, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 20.0, 0.0, 0, 0, 0, 0, 20, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 25.0, 3.0, 0, 0, M_PI, 0, 25.0, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 10.0, 3.0, 0, 0, M_PI, 0, 40.0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, timeStepSize));

    auto opt{std::make_shared<OptionalPredicateParameters>()};
    opt->trafficLightState = {TrafficLightState::red};
    opt->turningDirection = {TurningDirection::straight};
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {},
                                       opt)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_TRUE(
        pred.booleanEvaluation(1, world, obstacleOne, {}, opt)); // standing on stop line -> partially in intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, {}, opt)); // inside intersection
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, {}, opt)); // left intersection
    EXPECT_FALSE(pred.booleanEvaluation(
        0, world, obstacleTwo, {},
        opt)); // in front of intersection/traffic light with another direction -> completely on incoming
    EXPECT_FALSE(pred.booleanEvaluation(
        1, world, obstacleTwo, {},
        opt)); // standing on stop line -> partially in intersection, traffic light has another direction
}

TEST_F(AtRedTrafficLightPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(AtRedTrafficLightPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}