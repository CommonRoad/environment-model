#include "test_overtaking_bicycle_same_lane_predicate.h"
#include "../utils_predicate_test.h"

void TestOvertakingBicycleSameLanePredicate::SetUp() {
    std::shared_ptr<State> stateZeroObstacleEgo = std::make_shared<State>(0, 10, 1, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleEgo = std::make_shared<State>(1, 15, 1, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleEgo = std::make_shared<State>(2, 20, 3, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleEgo = std::make_shared<State>(3, 25, 3, 5, 0, 0);
    std::shared_ptr<State> stateFourObstacleEgo = std::make_shared<State>(4, 30, 3, 5, 0, 0);
    std::shared_ptr<State> stateFiveObstacleEgo = std::make_shared<State>(5, 35, 3, 5, 0, 0);
    std::shared_ptr<State> stateSixObstacleEgo = std::make_shared<State>(6, 40, 1, 5, 0, 0);
    std::shared_ptr<State> stateSevenObstacleEgo = std::make_shared<State>(7, 45, 1, 5, 0, 0);
    std::shared_ptr<State> stateEightObstacleEgo = std::make_shared<State>(8, 50, 1, 5, 0, 0);
    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleEgo)};
    obstacleEgo = std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleEgo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 15, 1, 3, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 18, 1, 3, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 21, 1, 3, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 24, 1, 3, 0, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 27, 1, 3, 0, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 30, 1, 3, 0, 0);
    std::shared_ptr<State> stateSixObstacleOne = std::make_shared<State>(6, 33, 1, 3, 0, 0);
    std::shared_ptr<State> stateSevenObstacleOne = std::make_shared<State>(7, 37, 1, 3, 0, 0);
    std::shared_ptr<State> stateEightObstacleOne = std::make_shared<State>(8, 40, 1, 3, 0, 0);
    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleOne),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleOne),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleOne)};
    obstacleOne =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::bicycle, 10,
                                            10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 2, 0.5));
}

TEST_F(TestOvertakingBicycleSameLanePredicate, BooleanEvaluationObjectsNoBicycle) {
    // only obstacleEgo, no bicycle
    auto roadNetwork{utils_predicate_test::create_road_network()};
    std::shared_ptr<World> world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo}, {}, 0.1));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(6, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(7, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(8, world, obstacleEgo));
}

TEST_F(TestOvertakingBicycleSameLanePredicate, BooleanEvaluationObjectsOvertakingBicycle) {
    // obsstacleEgo overtakes bicycle at timesteps 2, 3, and 4
    auto roadNetwork{utils_predicate_test::create_road_network()};
    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo}, {obstacleOne}, 0.1));

    //    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo));
    //    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo));
    //    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleEgo));
    //    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleEgo));
    //    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleEgo));
    //    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleEgo));
    //    EXPECT_FALSE(pred.booleanEvaluation(6, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(7, world, obstacleEgo));
    //    EXPECT_FALSE(pred.booleanEvaluation(8, world, obstacleEgo));
    //
    //    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleOne}, {obstacleEgo}, 0.1));
    //    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(6, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(7, world, obstacleOne));
    //    EXPECT_FALSE(pred.booleanEvaluation(8, world, obstacleOne));
}

TEST_F(TestOvertakingBicycleSameLanePredicate, RobustEvaluation) {
    auto roadNetwork{utils_predicate_test::create_road_network()};
    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo}, {obstacleOne}, 0.1));
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(TestOvertakingBicycleSameLanePredicate, ConstraintEvaluation) {
    auto roadNetwork{utils_predicate_test::create_road_network()};
    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo}, {obstacleOne}, 0.1));
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}