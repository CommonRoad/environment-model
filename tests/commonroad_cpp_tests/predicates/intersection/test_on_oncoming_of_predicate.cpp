#include "test_on_oncoming_of_predicate.h"
#include "commonroad_cpp/obstacle/state.h"

void OnOncomingOfPredicateTest::SetUp() {}

TEST_F(OnOncomingOfPredicateTest, BooleanEvaluationObjects) {}

TEST_F(OnOncomingOfPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(OnOncomingOfPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}
