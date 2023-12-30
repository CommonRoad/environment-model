//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_causes_braking_intersection_predicate.h"
#include "commonroad_cpp/obstacle/obstacle.h"

void CausesBrakingIntersectionPredicateTest::SetUp() {}

TEST_F(CausesBrakingIntersectionPredicateTest, BooleanEvaluation) {}

TEST_F(CausesBrakingIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world1, obstacleOne), std::runtime_error);
}

TEST_F(CausesBrakingIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world1, obstacleOne), std::runtime_error);
}