#include "test_keeps_lane_speed_limit_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void KeepsLaneSpeedLimitPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 30, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 45, 2, 35, 0, 0, 0, 45, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 95, 2, 45, 0, 0, 0, 95, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 150, 2, 30, 0, 0, 0, 150, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_2()};

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
}

TEST_F(KeepsLaneSpeedLimitPredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne));  // vehicle drives with lower velocity
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));  // vehicle drives exactly with the max speed
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne)); // vehicle drives too fast
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne));  // there exists no speed limit
}

TEST_F(KeepsLaneSpeedLimitPredicateTest, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_TRUE(
        pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat)); // vehicle drives with lower velocity
    EXPECT_EQ(stat->numExecutions, 1);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(1, world, obstacleOne, timer,
                                                stat)); // vehicle drives exactly with the max speed
    EXPECT_EQ(stat->numExecutions, 2);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(2, world, obstacleOne, timer, stat)); // vehicle drives too fast
    EXPECT_EQ(stat->numExecutions, 3);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(3, world, obstacleOne, timer, stat)); // there exists no speed limit
    EXPECT_EQ(stat->numExecutions, 4);
}

TEST_F(KeepsLaneSpeedLimitPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(KeepsLaneSpeedLimitPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
