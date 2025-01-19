#include "test_slow_other_vehicle_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void SlowOtherVehiclePredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 2, 2, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateZeroObstacleFive = std::make_shared<State>(0, 0, 6, 60, 0, 0, 0, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 12, 2, 2, 0, 0, 0, 12, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 22, 2, 50, 0, 0, 0, 22, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 32, 2, 36, 0, 0, 0, 32, 0);
    std::shared_ptr<State> stateOneObstacleFive = std::make_shared<State>(1, 60, 9.5, 0, 0, 0, 0, 60, 1.5);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 14, 2, 2, 0, 0, 0, 14, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 72, 2, 12, 0, 0, 0, 72, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 68, 2, 36, 0, 0, 0, 68, 0);
    std::shared_ptr<State> stateTwoObstacleFour = std::make_shared<State>(2, 44, 2, 50, 0, 0, 0, 44, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 16, 2, 2, 0, 0, 0, 16, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 84, 2, 2, 0, 0, 0, 84, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 98, 2, 36, 0, 0, 0, 98, 0);
    std::shared_ptr<State> stateThreeObstacleFour = std::make_shared<State>(3, 94, 2, 0, 0, 0, 0, 94, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(2, stateThreeObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    state_map_t trajectoryPredictionObstacleThree{std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree)};

    state_map_t trajectoryPredictionObstacleFour{std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFour)};

    state_map_t trajectoryPredictionObstacleFive{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFive)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateOneObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateOneObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    obstacleFour =
        std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, stateTwoObstacleFour, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));
    obstacleFive =
        std::make_shared<Obstacle>(Obstacle(5, ObstacleRole::DYNAMIC, stateZeroObstacleFive, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFive, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleOne},
                                          {obstacleTwo, obstacleThree, obstacleFour, obstacleFive}, 0.1));
}

TEST_F(SlowOtherVehiclePredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, obstacleFour));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, obstacleFive));
}

TEST_F(SlowOtherVehiclePredicateTest, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_FALSE(pred.statisticBooleanEvaluation(1, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_EQ(stat->numExecutions, 1);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(2, world, obstacleOne, timer, stat, obstacleThree));
    EXPECT_EQ(stat->numExecutions, 2);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(3, world, obstacleOne, timer, stat, obstacleFour));
    EXPECT_EQ(stat->numExecutions, 3);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(1, world, obstacleOne, timer, stat, obstacleFive));
    EXPECT_EQ(stat->numExecutions, 4);
}

TEST_F(SlowOtherVehiclePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(SlowOtherVehiclePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(SlowOtherVehiclePredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto world{
        std::make_shared<World>(World("testWorld", 0, std::get<1>(scenarioXml), std::get<0>(scenarioXml), {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(100)};

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs1, ego, {"5.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs1, ego, {"5.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(30, world, obs1, ego, {"5.0"}, true));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, obs1, {"5.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, ego, obs1, {"50.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, ego, obs1, {"5.0"}, true));
}
