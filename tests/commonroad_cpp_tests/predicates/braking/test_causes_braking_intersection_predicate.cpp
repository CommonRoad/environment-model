#include "test_causes_braking_intersection_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void CausesBrakingIntersectionPredicateTest::SetUp() {}

TEST_F(CausesBrakingIntersectionPredicateTest, BooleanEvaluation) {
    // TODO
}

TEST_F(CausesBrakingIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world1, obstacleOne), std::runtime_error);
}

TEST_F(CausesBrakingIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world1, obstacleOne), std::runtime_error);
}
