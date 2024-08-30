//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "test_on_similar_oriented_lanelet_without_type_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void OnSimilarOrientedLaneletWithoutTypePredicateTest::SetUp() {}

TEST_F(OnSimilarOrientedLaneletWithoutTypePredicateTest, BooleanEvaluationOnShoulder) {}

TEST_F(OnSimilarOrientedLaneletWithoutTypePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(OnSimilarOrientedLaneletWithoutTypePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
