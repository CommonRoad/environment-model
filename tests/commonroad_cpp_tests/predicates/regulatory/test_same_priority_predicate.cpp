//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_same_priority_predicate.h"
#include "../utils_predicate_test.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

TEST_F(SamePriorityPredicateTest, BooleanEvaluation) {
    // Todo
}

TEST_F(SamePriorityPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(SamePriorityPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}