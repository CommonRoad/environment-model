//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_makes_u_turn_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <math.h>

void MakesUTurnPredicateTest::SetUp() {

    double curvlinOrientation0 = 0;
    double curvlinOrientation1 = M_PI / 8;
    double curvlinOrientation2 = M_PI / 2;
    double curvlinOrientation3 = (3 * M_PI) / 4;

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, curvlinOrientation0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 0, 10, 0, 0, curvlinOrientation1, 10, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 0, 10, 0, 0, curvlinOrientation2, 20, 0);
    std::shared_ptr<State> stateThreeEgoVehicle =
        std::make_shared<State>(3, 30, 0, 10, 0, 0, curvlinOrientation3, 30, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
    };

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(MakesUTurnPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, egoVehicle));
}

TEST_F(MakesUTurnPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(MakesUTurnPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
