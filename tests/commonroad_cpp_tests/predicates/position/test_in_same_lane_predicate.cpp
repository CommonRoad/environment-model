//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_in_same_lane_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/obstacle/state.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestInSameLanePredicate::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 10, 2, 10, -1, 0, 0, 10, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 4, 10, 0, 0, 0, 10, 2);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 20, 2, 10, 0, 0, 0, 20, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 30, 4, 10, 0, 0, 0, 30, 2);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 2, 10, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 40, 6, 10, 0, 0, 0, 40, 4);

    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 40, 2, 10, 0, 0, 0, 40, 0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(4, 20, 2, 10, 0, 0, 0, 20, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(4, stateZeroObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                    std::vector<std::shared_ptr<Obstacle>>{obstacleTwo, obstacleThree}, 0.1);
}

TEST_F(TestInSameLanePredicate, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, obstacleTwo));  // vehicles completely on same lane
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo));  // ego vehicle partially in another lane
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo));  // other vehicle partially in another lane
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, obstacleTwo)); // vehicles not in same lane
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleOne,
                                       obstacleThree)); // vehicles completely on same lane, but other vehicle is behind
}

TEST_F(TestInSameLanePredicate, StatisticBooleanEvaluation) {
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(0, world, obstacleOne, obstacleTwo)); // vehicles completely on same lane
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(1, world, obstacleOne, obstacleTwo)); // ego vehicle partially in another lane
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(2, world, obstacleOne, obstacleTwo)); // other vehicle partially in another lane
    EXPECT_FALSE(pred.statisticBooleanEvaluation(3, world, obstacleOne, obstacleTwo)); // vehicles not in same lane
    EXPECT_TRUE(pred.statisticBooleanEvaluation(
        4, world, obstacleOne,
        obstacleThree)); // vehicles completely on same lane, but other vehicle is behind
}

TEST_F(TestInSameLanePredicate, BooleanEvaluationObjectsInIntersection) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    std::vector<std::shared_ptr<Obstacle>> egoObstacles{
        obstacle_operations::getObstacleById(obstaclesScenarioOne, 1219)};
    std::vector<std::shared_ptr<Obstacle>> relevantObstacles{
        obstacle_operations::getObstacleById(obstaclesScenarioOne, 1230),
        obstacle_operations::getObstacleById(obstaclesScenarioOne, 1214)};
    auto world{std::make_shared<World>(0, roadNetworkScenarioOne, egoObstacles, relevantObstacles, timeStepSizeOne)};
    EXPECT_FALSE(pred.booleanEvaluation(
        0, world, egoObstacles.at(0),
        relevantObstacles.at(0))); // vehicles on right turning lane, but ego vehicle drives straight
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoObstacles.at(0),
                                       relevantObstacles.at(1))); // vehicle on straight lane and ego vehicle drives
                                                                  // straight
}

TEST_F(TestInSameLanePredicate, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(TestInSameLanePredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}