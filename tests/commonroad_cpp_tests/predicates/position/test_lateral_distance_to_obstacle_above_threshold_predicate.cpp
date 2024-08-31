#include "test_lateral_distance_to_obstacle_above_threshold_predicate.h"
#include "../utils_predicate_test.h"

#include <commonroad_cpp/world.h>

#include <gtest/gtest.h>

#include <set>
#include <stdexcept>

void TestLateralDistanceToObstacleAboveThresholdPredicate::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 50, 1, 20, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 50, 1, 20, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 50, 1, 20, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 50, 1, 20, 0, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 50, 1, 20, 0, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 50, 1, 20, 0, 0);
    std::shared_ptr<State> stateSixObstacleOne = std::make_shared<State>(6, 50, 1, 20, 0, 0);

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 40, 1, 25, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 43.25, 1, 25, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 43.75, 1, 25, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 50, 4, 25, 0, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 52.99, 4.75, 25, 0, 0);
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 50, 5.5, 25, 0, 0);
    std::shared_ptr<State> stateSixEgoVehicle = std::make_shared<State>(6, 47.01, 4, 25, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(6, stateSixEgoVehicle)};

    obstacleOne =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::bicycle, 50,
                                            10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 2, 2));
    egoVehicle = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
}

TEST_F(TestLateralDistanceToObstacleAboveThresholdPredicate, Urban) {
    auto roadNetwork1{utils_predicate_test::create_road_network_with_2_successors(
        {LaneletType::urban}, {LaneletType::urban}, {LaneletType::urban}, {LaneletType::urban})};
    auto world = std::make_shared<World>(World("testWorld", 0, roadNetwork1, {egoVehicle}, {obstacleOne}, 0.1));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, obstacleOne, {dMinUrban}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, obstacleOne, {dMinUrban}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, obstacleOne, {dMinUrban}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, obstacleOne, {dMinUrban}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle, obstacleOne, {dMinUrban}));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, egoVehicle, obstacleOne, {dMinUrban}));
    EXPECT_FALSE(pred.booleanEvaluation(6, world, egoVehicle, obstacleOne, {dMinUrban}));
}

TEST_F(TestLateralDistanceToObstacleAboveThresholdPredicate, Interstate) {
    auto roadNetwork2{utils_predicate_test::create_road_network_with_2_successors(
        {LaneletType::interstate}, {LaneletType::interstate}, {LaneletType::interstate}, {LaneletType::interstate})};
    auto world = std::make_shared<World>(World("testWorld", 0, roadNetwork2, {egoVehicle}, {obstacleOne}, 0.1));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, obstacleOne, {dMinNonUrban}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, obstacleOne, {dMinNonUrban}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, obstacleOne, {dMinNonUrban}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, obstacleOne, {dMinNonUrban}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, obstacleOne, {dMinNonUrban}));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, egoVehicle, obstacleOne, {dMinNonUrban}));
    EXPECT_FALSE(pred.booleanEvaluation(6, world, egoVehicle, obstacleOne, {dMinNonUrban}));
}

void TestLateralDistanceToObstacleAboveThresholdPredicate::setUpLeft() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 4.75, 10, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 4.75, 10, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 4, 10, 0, 0);

    std::shared_ptr<State> stateZeroObstacleOne =
        std::make_shared<State>(0, 30, 7, 10, 0, boost::math::constants::pi<double>());
    std::shared_ptr<State> stateOneObstacleOne =
        std::make_shared<State>(1, 30, 7, 10, 0, boost::math::constants::pi<double>());
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 7, 10, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 7, 10, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle)};

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    egoVehicle2 = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    obstacleOne2 = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateTwoObstacleOne, ObstacleType::car,
                                                       50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    world2 = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle2}, {obstacleOne2}, 0.1));
}

void TestLateralDistanceToObstacleAboveThresholdPredicate::setUpRight() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 3.25, 10, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 3.25, 10, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 4, 10, 0, 0);

    std::shared_ptr<State> stateZeroObstacleOne =
        std::make_shared<State>(0, 20, 1, 10, 0, boost::math::constants::pi<double>());
    std::shared_ptr<State> stateOneObstacleOne =
        std::make_shared<State>(1, 30, 1, 10, 0, boost::math::constants::pi<double>());
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 1, 10, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 1, 10, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle)};

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    egoVehicle2 = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    obstacleOne2 = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateTwoObstacleOne, ObstacleType::car,
                                                       50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    world2 = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle2}, {obstacleOne2}, 0.1));
}

TEST_F(TestLateralDistanceToObstacleAboveThresholdPredicate, BooleanEvaluationObjectsLeft) {
    setUpLeft();
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
    EXPECT_TRUE(pred.booleanEvaluation(3, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
}

TEST_F(TestLateralDistanceToObstacleAboveThresholdPredicate, BooleanEvaluationObjectsRight) {
    setUpRight();
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
    EXPECT_TRUE(pred.booleanEvaluation(3, world2, egoVehicle2, {obstacleOne2}, {closeToOtherVehicle}));
}

TEST_F(TestLateralDistanceToObstacleAboveThresholdPredicate, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, {}, obstacleOne, egoVehicle), std::runtime_error);
}

TEST_F(TestLateralDistanceToObstacleAboveThresholdPredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, {}, obstacleOne, egoVehicle), std::runtime_error);
}
