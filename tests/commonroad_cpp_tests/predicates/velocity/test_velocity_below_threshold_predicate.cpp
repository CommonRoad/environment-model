#include "test_velocity_below_threshold_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void VelocityBelowThresholdPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 1, 0, 0, 0, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 1, 2, 0, 0, 0, 0, 1, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 1, 2, 0.001, 0, 0, 0, 1, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 1.001, 2, 3, 0, 0, 0, 1.001, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleOne}, {}, 0.1));
}

TEST_F(VelocityBelowThresholdPredicateTest, BooleanEvaluationObjects) {
    std::vector<std::string> opt{"0"};
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne, {}, opt));
    opt = {"1"};
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, {}, opt));
    opt = {"2"};
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, {}, opt));
    opt = {"0"};
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, {}, opt));
}

TEST_F(VelocityBelowThresholdPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(VelocityBelowThresholdPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(VelocityBelowThresholdPredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(100)};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, {}, {"5.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs1, {}, {"5.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(30, world, obs1, {}, {"5.0"}, true));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, {}, {"5.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, ego, {}, {"50.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, ego, {}, {"5.0"}, true));
}
