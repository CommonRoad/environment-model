//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_relevant_traffic_light_predicate.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"

TEST_F(RelevantRedTrafficLightPredicateTest, BooleanEvaluation) {}

TEST_F(RelevantRedTrafficLightPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(RelevantRedTrafficLightPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}