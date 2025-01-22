#include "test_adjacent_lanelet_of_type_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void AdjacentLaneletOfTypePredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 6, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 10, 0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(3, 30, 6, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(4, 40, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(5, 50, 2, 10, 0, 0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(6, 60, 6, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(7, 70, 4, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(8, 80, 2, 10, 0, 0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};
    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(4, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(5, stateTwoObstacleTwo)};
    state_map_t trajectoryPredictionObstacleThree{std::pair<int, std::shared_ptr<State>>(7, stateOneObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(8, stateTwoObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));

    auto roadNetworkOne{
        utils_predicate_test::create_road_network({LaneletType::interstate, LaneletType::mainCarriageWay},
                                                  {LaneletType::interstate, LaneletType::mainCarriageWay})};
    auto roadNetworkTwo{
        utils_predicate_test::create_road_network({}, {LaneletType::interstate, LaneletType::mainCarriageWay})};
    auto roadNetworkThree{utils_predicate_test::create_road_network(
        {LaneletType::interstate, LaneletType::shoulder}, {LaneletType::interstate, LaneletType::mainCarriageWay})};
    worldOne = std::make_shared<World>(World("testWorld", 0, roadNetworkOne, {obstacleOne}, {}, 0.1));
    worldTwo = std::make_shared<World>(World("testWorld", 0, roadNetworkTwo, {obstacleTwo}, {}, 0.1));
    worldThree = std::make_shared<World>(World("testWorld", 0, roadNetworkThree, {obstacleThree}, {}, 0.1));
}

TEST_F(AdjacentLaneletOfTypePredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, worldOne, obstacleOne, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOne, obstacleOne, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOne, obstacleOne, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldTwo, obstacleTwo, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldTwo, obstacleTwo, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, worldTwo, obstacleTwo, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(6, worldThree, obstacleThree, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(7, worldThree, obstacleThree, {}, {"right", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(8, worldThree, obstacleThree, {}, {"right", "mainCarriageWay"}));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldOne, obstacleOne, {}, {"left", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOne, obstacleOne, {}, {"left", "mainCarriageWay"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOne, obstacleOne, {}, {"left", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldTwo, obstacleTwo, {}, {"left", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldTwo, obstacleTwo, {}, {"left", "mainCarriageWay"}));
    EXPECT_TRUE(pred.booleanEvaluation(5, worldTwo, obstacleTwo, {}, {"left", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(6, worldThree, obstacleThree, {}, {"left", "mainCarriageWay"}));
    EXPECT_FALSE(pred.booleanEvaluation(7, worldThree, obstacleThree, {}, {"left", "mainCarriageWay"}));
    EXPECT_TRUE(pred.booleanEvaluation(8, worldThree, obstacleThree, {}, {"left", "mainCarriageWay"}));
}

TEST_F(AdjacentLaneletOfTypePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, worldOne, obstacleOne), std::runtime_error);
}

TEST_F(AdjacentLaneletOfTypePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, worldOne, obstacleOne), std::runtime_error);
}

TEST_F(AdjacentLaneletOfTypePredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(100)};
    auto obs2{world->findObstacle(101)};
    auto obs3{world->findObstacle(102)};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs2, {}, {"right", "interstate"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs3, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, ego, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs1, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs2, {}, {"right", "interstate"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs3, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, ego, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, obs1, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, obs2, {}, {"right", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, obs3, {}, {"right", "interstate"}, true));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs2, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs3, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, ego, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs1, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs2, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs3, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, ego, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, obs1, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, obs2, {}, {"left", "interstate"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, obs3, {}, {"left", "interstate"}, true));
}
