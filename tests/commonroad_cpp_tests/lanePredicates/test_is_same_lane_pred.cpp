#include "test_is_same_lane_pred.h"
#include "../predicates/utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestIsSameLanePredicate::SetUp() {
    auto roadNetwork{utils_predicate_test::create_road_network()};
    world = std::make_shared<World>("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{},
                                    std::vector<std::shared_ptr<Obstacle>>{}, 0.1);
    laneOne = world.get()->getRoadNetwork()->getLanes().at(0);
    laneTwo = world.get()->getRoadNetwork()->getLanes().at(1);
}

TEST_F(TestIsSameLanePredicate, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, laneOne, laneOne));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, laneOne, laneTwo));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, laneTwo, laneOne));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, laneTwo, laneTwo));
}

TEST_F(TestIsSameLanePredicate, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, laneOne, timer, stat, laneOne));
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, laneOne, timer, stat, laneTwo));
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, laneTwo, timer, stat, laneOne));
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, laneTwo, timer, stat, laneTwo));
}
