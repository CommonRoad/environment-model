//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_main_carriageway_right_lane_predicate.h"
#include "../utils_predicate_test.h"

void MainCarriagewayRightLanePredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 6, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 10, 0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(3, 30, 6, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(4, 40, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(5, 50, 2, 10, 0, 0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(6, 60, 6, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(7, 70, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(8, 80, 2, 10, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};
    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(4, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(5, stateTwoObstacleTwo)};
    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(7, stateOneObstacleThree),
        std::pair<int, std::shared_ptr<State>>(8, stateTwoObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));

    auto roadNetworkOne{
        utils_predicate_test::create_road_network({LaneletType::interstate, LaneletType::mainCarriageWay},
                                                  {LaneletType::interstate, LaneletType::mainCarriageWay})};
    auto roadNetworkTwo{
        utils_predicate_test::create_road_network({}, {LaneletType::interstate, LaneletType::mainCarriageWay})};
    auto roadNetworkThree{utils_predicate_test::create_road_network(
        {LaneletType::interstate, LaneletType::shoulder}, {LaneletType::interstate, LaneletType::mainCarriageWay})};
    worldOne = std::make_shared<World>(World("testWorld", 0, roadNetworkOne, {obstacleOne}, {}, 0.1));
    worldTwo = std::make_shared<World>(World("testWorld", 0, roadNetworkTwo, {obstacleTwo}, {}, 0.1));
    worldThree = std::make_shared<World>(World("testWorld", 0, roadNetworkThree, {obstacleThree}, {}, 0.1));
}

TEST_F(MainCarriagewayRightLanePredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, worldOne, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOne, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOne, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldTwo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldTwo, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(5, worldTwo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(6, worldThree, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(7, worldThree, obstacleThree));
    EXPECT_FALSE(pred.booleanEvaluation(8, worldThree, obstacleThree));
}

TEST_F(MainCarriagewayRightLanePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, worldOne, obstacleOne), std::runtime_error);
}

TEST_F(MainCarriagewayRightLanePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, worldOne, obstacleOne), std::runtime_error);
}