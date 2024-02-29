#include "test_right_of_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void RightOfPredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 5, 6, 5, 0, 0, 0, 5, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 2, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 6, 11, 0, 0, 0, 10, 4);

    /* State 2 */
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 21, 6, 8, 0, 0, 0, 21, 4);

    /* State 3 */
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 2, 10, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 29, 6, 8, 0, 0, 0, 29, 4);

    /* State 4 */
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 50, 2, 10, 0, 0, 0, 50, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 55, 6, 10, 0, 0, 0, 55, 0);

    /* State 5 */
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 60, 2, 10, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 65, -2, 5, 0, 0, 0, 65, -4);

    /* State 6 */
    std::shared_ptr<State> stateSixObstacleOne = std::make_shared<State>(6, 70, 2, 10, 0, 0, 0, 70, 0);
    std::shared_ptr<State> stateSixEgoVehicle = std::make_shared<State>(6, 70, -2, 11, 0, 0, 0, 70, -4);

    /* State 7 */
    std::shared_ptr<State> stateSevenObstacleOne = std::make_shared<State>(7, 80, 2, 10, 0, 0, 0, 80, 0);
    std::shared_ptr<State> stateSevenEgoVehicle = std::make_shared<State>(8, 81, -2, 8, 0, 0, 0, 81, -4);

    /* State 8 */
    std::shared_ptr<State> stateEightObstacleOne = std::make_shared<State>(8, 90, 2, 10, 0, 0, 0, 90, 0);
    std::shared_ptr<State> stateEightEgoVehicle = std::make_shared<State>(8, 89, -2, 10, 0, 0, 0, 89, -4);

    /* State 9 */
    std::shared_ptr<State> stateNineObstacleOne = std::make_shared<State>(9, 100, 2, 10, 0, 0, 0, 100, 0);
    std::shared_ptr<State> stateNineObstacleTwo = std::make_shared<State>(9, 100, 6, 10, 0, 0, 0, 100, 4);

    /* State 10 */
    std::shared_ptr<State> stateTenObstacleOne = std::make_shared<State>(10, 110, 2, 10, 0, 0, 0, 110, 0);
    std::shared_ptr<State> stateTenObstacleTwo = std::make_shared<State>(10, 110, -2, 10, 0, 0, 0, 110, -4);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleOne),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleOne),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleOne),
        std::pair<int, std::shared_ptr<State>>(9, stateNineObstacleOne),
        std::pair<int, std::shared_ptr<State>>(10, stateTenObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(6, stateSixEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(8, stateEightEgoVehicle)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(10, stateTenObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    egoVehicle = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 4.9, 2));
    obstacleTwo =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateNineObstacleTwo, ObstacleType::truck, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleTwo, 8, 4.25));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {obstacleOne, obstacleTwo}, 0.1));
}

TEST_F(RightOfPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(
        pred.booleanEvaluation(0, world, obstacleOne, egoVehicle)); // other vehicle in right lane but no adjacent
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, egoVehicle)); // other vehicle exactly right of
    EXPECT_TRUE(
        pred.booleanEvaluation(2, world, obstacleOne, egoVehicle)); // other vehicle partially right of in behind
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, egoVehicle)); // other vehicle partially right of in front
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleOne, egoVehicle)); // other vehicle in same lane in behind
    EXPECT_FALSE(
        pred.booleanEvaluation(5, world, obstacleOne, egoVehicle)); // other vehicle in left lane but not adjacent
    EXPECT_FALSE(pred.booleanEvaluation(6, world, obstacleOne, egoVehicle)); // other vehicle exactly left of
    EXPECT_FALSE(
        pred.booleanEvaluation(7, world, obstacleOne, egoVehicle)); // other vehicle partially left of in behind
    EXPECT_FALSE(pred.booleanEvaluation(8, world, obstacleOne, egoVehicle)); // other vehicle partially left of in front
    EXPECT_TRUE(
        pred.booleanEvaluation(9, world, obstacleOne, obstacleTwo)); // other vehicle right of in front and behind
    EXPECT_FALSE(
        pred.booleanEvaluation(10, world, obstacleOne, obstacleTwo)); // other vehicle partially left of in front
}

TEST_F(RightOfPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleTwo, egoVehicle), std::runtime_error);
}

TEST_F(RightOfPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleTwo, egoVehicle), std::runtime_error);
}