//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_approach_four_way_stop_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void ApproachFourWayStopPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() +
                               "/predicates/USA_Test4WayStopIntersection-1_1_T.1.xml"}; // this has STOP signs
    std::string pathToTestFile2{
        TestUtils::getTestScenarioDirectory() +
        "/predicates/DEU_TrafficLightTest-1/DEU_TrafficLightTest-1_1_T-1.pb"}; // this has no STOP signs
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    const auto &[obstacles2, roadNetwork2, timeStepSize2] = InputUtils::getDataFromCommonRoad(pathToTestFile2);

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 25, 3, 10, 0, -M_PI / 2, 0, 25, 3);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 23.5, 15, 10, 0, -M_PI / 2, 0, 23.5, 15);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 28.6, 3.5, 10, 0, M_PI, 0, 10, 3.5);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 43, 3, 10, 0, M_PI, 0, 10, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
    world_2 = std::make_shared<World>(World("testWorld", 0, roadNetwork2, {egoVehicle}, {}, 0.1));
}

TEST_F(ApproachFourWayStopPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(0, world_2, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(3, world_2, egoVehicle));
}

TEST_F(ApproachFourWayStopPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(ApproachFourWayStopPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}