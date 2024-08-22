
#include "test_close_lateral_distance_to_obstacle_predicate.h"
#include "../utils_predicate_test.h"

void CloseLateralDistanceToObstaclePredicateTest::setUpLeft() {

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

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateTwoObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));

    world2 = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {obstacleOne}, 0.1));
}

void CloseLateralDistanceToObstaclePredicateTest::setUpRight() {

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

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateTwoObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));

    world2 = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {obstacleOne}, 0.1));
}

TEST_F(CloseLateralDistanceToObstaclePredicateTest, BooleanEvaluationObjectsRight1) {
    setUpLeft();
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, egoVehicle, {obstacleOne}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, egoVehicle, {obstacleOne}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, egoVehicle, {obstacleOne}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, egoVehicle, {obstacleOne}, {"right"}));
}

TEST_F(CloseLateralDistanceToObstaclePredicateTest, BooleanEvaluationObjectsLeft1) {
    setUpLeft();
    EXPECT_TRUE(pred.booleanEvaluation(0, world2, egoVehicle, {obstacleOne}, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, egoVehicle, {obstacleOne}, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, egoVehicle, {obstacleOne}, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, egoVehicle, {obstacleOne}, {"left"}));
}

TEST_F(CloseLateralDistanceToObstaclePredicateTest, BooleanEvaluationObjectsRight2) {
    setUpRight();
    EXPECT_TRUE(pred.booleanEvaluation(0, world2, egoVehicle, {obstacleOne}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, egoVehicle, {obstacleOne}, {"right"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, egoVehicle, {obstacleOne}, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, egoVehicle, {obstacleOne}, {"right"}));
}

TEST_F(CloseLateralDistanceToObstaclePredicateTest, BooleanEvaluationObjectsLeft2) {
    setUpRight();
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, egoVehicle, {obstacleOne}, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, egoVehicle, {obstacleOne}, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, egoVehicle, {obstacleOne}, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, egoVehicle, {obstacleOne}, {"left"}));
}

TEST_F(CloseLateralDistanceToObstaclePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(CloseLateralDistanceToObstaclePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
