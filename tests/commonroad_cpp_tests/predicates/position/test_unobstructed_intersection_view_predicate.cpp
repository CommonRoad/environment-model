//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_unobstructed_intersection_view_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void UnobstructedIntersectionViewPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 1000, 1006.9, 10, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 1010, 1006.5, 10, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 1020, 1004.75, 10, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 1030, 1004, 10, 0, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 1040, 1006, 10, 0, 0);

    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroEgoVehicle, ObstacleType::car, 50, 10, 3, -10,
                                                     0.3, trajectoryPredictionEgoVehicle, 5, 2));

    setUpIncoming();
    setUpIntersection();

    auto roadNetwork = std::make_shared<RoadNetwork>(
        RoadNetwork(lanelets, SupportedTrafficSignCountry::GERMANY, {}, {}, {intersection1}));
    world = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(UnobstructedIntersectionViewPredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle));
    std::vector<vertex> fovSmall{{0.0, -5.0}, {5 * cos(M_PI * (7 / 4)), 5 * sin(M_PI * (7 / 4))},
                                 {5.0, 0.0},  {5 * cos(M_PI * (1 / 4)), 5 * sin(M_PI * (1 / 4))},
                                 {0.0, 5.0},  {5 * cos(M_PI * (3 / 4)), 5 * sin(M_PI * (3 / 4))},
                                 {-5.0, 0.0}, {5 * cos(M_PI * (5 / 4)), 5 * sin(M_PI * (5 / 4))},
                                 {0.0, -5.0}};
    egoVehicle->setFov(geometric_operations::rotateAndTranslateVertices(
        fovSmall, {egoVehicle->getCurrentState()->getXPosition(), egoVehicle->getCurrentState()->getYPosition()}, 0));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle));
}

TEST_F(UnobstructedIntersectionViewPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(UnobstructedIntersectionViewPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}