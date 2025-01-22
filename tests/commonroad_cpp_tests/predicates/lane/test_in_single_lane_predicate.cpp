#include "test_in_single_lane_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestInSingleLanePredicate::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, 0, 0, 0, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 4, 10, 0, 0, 0, 10, 2);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 2, 10, 0, 0, 0, 30, 0);

    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 40, 2, 10, 0, 0, 0, 40, 0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                    std::vector<std::shared_ptr<Obstacle>>{}, 0.1);
}

TEST_F(TestInSingleLanePredicate, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne)); // ego vehicle partially in two lanes
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleOne));
}

TEST_F(TestInSingleLanePredicate, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat));
    EXPECT_FALSE(
        pred.statisticBooleanEvaluation(1, world, obstacleOne, timer, stat)); // ego vehicle partially in two lanes
    EXPECT_TRUE(pred.statisticBooleanEvaluation(2, world, obstacleOne, timer, stat));
    EXPECT_TRUE(pred.statisticBooleanEvaluation(3, world, obstacleOne, timer, stat));
    EXPECT_TRUE(pred.statisticBooleanEvaluation(4, world, obstacleOne, timer, stat));
}

TEST_F(TestInSingleLanePredicate, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(TestInSingleLanePredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(TestInSingleLanePredicate, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto obs1{world->findObstacle(100)};
    auto obs2{world->findObstacle(101)};
    auto obs3{world->findObstacle(102)};

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs1, {}, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs1, {}, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, obs1, {}, {}, true));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs2, {}, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs2, {}, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, obs2, {}, {}, true));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs3, {}, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs3, {}, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, obs3, {}, {}, true));
}
