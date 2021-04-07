//
// Created by sebastian on 26.12.20.
//

#include "test_safe_distance_predicate.h"
#include "obstacle/state.h"

void SafeDistancePredicateTest::SetUp() {
    std::shared_ptr<State> stateEgoZero = std::make_shared<State>(0, 0, 0, 20, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOtherZero = std::make_shared<State>(0, 20, 0, 20, -1, 0, 0, 20, 0);
    std::shared_ptr<State> stateEgoOne = std::make_shared<State>(1, 20, 0, 20, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateOtherOne = std::make_shared<State>(1, 30, 0, 0, 0, 0, 0, 30, 0);

    std::map<int, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
            std::pair<int, std::shared_ptr<State>>(0, stateEgoZero),
            std::pair<int, std::shared_ptr<State>>(1, stateEgoOne)};
    std::map<int, std::shared_ptr<State>> trajectoryPredictionOtherVehicle{
            std::pair<int, std::shared_ptr<State>>(0, stateOtherZero),
            std::pair<int, std::shared_ptr<State>>(1, stateOtherOne)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(0, false, stateEgoZero, ObstacleType::car, 50, 10, 3, -10, 0.3,
                                                     trajectoryPredictionEgoVehicle, 5, 2));
    otherVehicle = std::make_shared<Obstacle>(
            Obstacle(1, false, stateOtherZero, ObstacleType::car, 50, 10, 3, -10, 0.3,
                     trajectoryPredictionOtherVehicle, 5, 2));
}

TEST_F(SafeDistancePredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.Predicate::booleanEvaluation(0, {}, egoVehicle, otherVehicle));
    EXPECT_FALSE(pred.Predicate::booleanEvaluation(1, {}, egoVehicle, otherVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(0, 20, 20, 20, -10, -10, 0.3));
    EXPECT_FALSE(pred.booleanEvaluation(20, 30, 20, 0, -10, -10, 0.3));
}

TEST_F(SafeDistancePredicateTest, ConstraintEvaluation) {
    EXPECT_NEAR(pred.constraintEvaluation(0, {}, egoVehicle, otherVehicle).realValuedConstraint, 6.0, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(1, {}, egoVehicle, otherVehicle).realValuedConstraint, 26.0, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(20, 20, -10, -10, 0.3).realValuedConstraint, 6.0, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(20, 0, -10, -10, 0.3).realValuedConstraint, 26.0, 0.001);
}

TEST_F(SafeDistancePredicateTest, RobustEvaluation) {
    EXPECT_NEAR(pred.robustEvaluation(0, {}, egoVehicle, otherVehicle), 9, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(1, {}, egoVehicle, otherVehicle), -21, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(2.5, 17.5, 20, 20, -10, -10, 0.3), 9, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(22.5, 27.5, 20, 0, -10, -10, 0.3), -21, 0.001);
}

TEST_F(SafeDistancePredicateTest, StatisticBooleanEvaluation) {
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, {}, egoVehicle, otherVehicle));
    EXPECT_EQ(SafeDistancePredicate::statistics.numExecutions, 1);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(1, {}, egoVehicle, otherVehicle));
    EXPECT_EQ(SafeDistancePredicate::statistics.numExecutions, 2);
}

TEST_F(SafeDistancePredicateTest, ComputeSafeDistance) {
    // both vehicles standing, no reaction time
    EXPECT_NEAR(SafeDistancePredicate::computeSafeDistance(0, 0, -10, -10, 0.0), 0, 0.001);
    // both vehicles same velocity, no reaction time
    EXPECT_NEAR(SafeDistancePredicate::computeSafeDistance(5, 5, -10, -10, 0.0), 0, 0.001);
    // both vehicles same velocity, with reaction time
    EXPECT_NEAR(SafeDistancePredicate::computeSafeDistance(5, 5, -10, -10, 10.0), 50.0, 0.001);
    // following vehicle higher velocity, no reaction time
    EXPECT_NEAR(SafeDistancePredicate::computeSafeDistance(10, 0, -10, -10, 0.0), 5.0, 0.001);
    // leading vehicle higher velocity, no reaction time
    EXPECT_NEAR(SafeDistancePredicate::computeSafeDistance(0, 10, -10, -10, 0.0), -5.0, 0.001);
}
