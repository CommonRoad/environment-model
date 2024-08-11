#include "test_in_standstill_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void InStandstillPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 1, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 1, 2, 0, 0, 0, 0, 1, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 1, 2, 0.001, 0, 0, 0, 1, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 1.001, 2, -0.001, 0, 0, 0, 1.001, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
}

TEST_F(InStandstillPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // drives with too high velocity
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));  // vehicle drives exactly zero speed
    EXPECT_TRUE(pred.booleanEvaluation(2, world,
                                       obstacleOne)); // drives with speed slightly below upper standstill error margin
    EXPECT_TRUE(pred.booleanEvaluation(3, world,
                                       obstacleOne)); // drives with speed slightly above upper standstill error margin
}

TEST_F(InStandstillPredicateTest, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat)); // drives with too high velocity
    EXPECT_EQ(stat->numExecutions, 1);
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(1, world, obstacleOne, timer, stat)); // vehicle drives exactly zero speed
    EXPECT_EQ(stat->numExecutions, 2);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(
        2, world, obstacleOne, timer, stat)); // drives with speed slightly below upper standstill error margin
    EXPECT_EQ(stat->numExecutions, 3);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(
        3, world, obstacleOne, timer, stat)); // drives with speed slightly above upper standstill error margin
    EXPECT_EQ(stat->numExecutions, 4);
}

TEST_F(InStandstillPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(InStandstillPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
