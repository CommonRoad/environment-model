//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_in_intersection_conflict_area_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/geometry/geometric_operations.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"

void InIntersectionConflictAreaPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 1009.5, 950, 10, 0, 1.5708);
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 1009.5, 995, 10, 0, 1.5708);

    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 1009.5, 990, 10, 0, 1.5708);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 1009.5, 999, 10, 0, 1.5708);

    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 1009.5, 995, 10, 0, 1.5708);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 1007.5, 1007.5, 10, 0, 3.14);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                                         std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle)};
    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
    };

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroEgoVehicle, ObstacleType::car, 50, 10, 3, -10,
                                                     0.3, trajectoryPredictionEgoVehicle, 5, 2));

    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, false, stateZeroObstacleOne, ObstacleType::car, 50, 10, 3, -10,
                                                      0.3, trajectoryPredictionObstacleOne, 5, 2));

    setUpIncoming();
    setUpIntersection();

    auto roadNetwork = std::make_shared<RoadNetwork>(
        RoadNetwork(lanelets, SupportedTrafficSignCountry::GERMANY, {}, {}, {intersection1}));
    world = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(InIntersectionConflictAreaPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, obstacleOne));

    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle, obstacleOne));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, obstacleOne));
}

TEST_F(InIntersectionConflictAreaPredicateTest, TestScenario1) {
    std::array<std::string, 1> scenarios{"DEU_test_turn_left_2.xml"};
    for (const auto &scen : scenarios) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/predicates/" + scen};
        const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
            InputUtils::getDataFromCommonRoad(pathToTestFileOne);
        auto world{
            std::make_shared<World>(World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne))};

        EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_FALSE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_TRUE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_TRUE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_FALSE(pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));
        EXPECT_TRUE(
            pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1))); // todo check
    }
}

TEST_F(InIntersectionConflictAreaPredicateTest, TestScenario2) {
    std::array<std::string, 1> scenarios{"DEU_test_right_before_left_2.xml"};
    for (const auto &scen : scenarios) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/predicates/" + scen};
        const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
            InputUtils::getDataFromCommonRoad(pathToTestFileOne);
        auto world{
            std::make_shared<World>(World(0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne))};

        EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_FALSE(pred.booleanEvaluation(37, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_TRUE(pred.booleanEvaluation(37, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_TRUE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_FALSE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_TRUE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
        EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

        EXPECT_FALSE(pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));
        EXPECT_FALSE(pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    }
}

TEST_F(InIntersectionConflictAreaPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(InIntersectionConflictAreaPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}