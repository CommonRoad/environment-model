#include "test_two_or_more_lanes_for_one_dir_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void TwoOrMoreLanesPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 2, 10, 0, 0, 0, 0, 2);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 2, 10, 0, 0, 0, 10, 2);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 6, 10, 0, 0, 0, 20, 6);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 6, 10, 0, 0, 0, 30, 6);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 30, 10, 10, 0, 0, 0, 30, 10);
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 30, 10, 10, 0, 0, 0, 30, 10);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(4, stateThreeEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(5, stateThreeEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_4()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(TwoOrMoreLanesPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, egoVehicle));
}

TEST_F(TwoOrMoreLanesPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(TwoOrMoreLanesPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(TwoOrMoreLanesPredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto obs1{world->findObstacle(100)};

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs1, {}, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs1, {}, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(25, world, obs1, {}, {}, true));
}
