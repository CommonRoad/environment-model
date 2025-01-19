#include "test_drives_with_slightly_higher_speed_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void DrivesWithSlightlyHigherSpeedPredicateTest::SetUp() {
    /* State 0 */
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 5, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 10, 0, 10, 0, 0, 0, 10, 0);

    /* State 1 */
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 5, 0, 10, 0, 0, 0, 5, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 20, 0, 10, 0, 0, 0, 20, 0);

    /* State 2 */
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 15, 0, 15, 0, 0, 0, 15, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 30, 0, 10, 0, 0, 0, 30, 0);

    /* State 3 */
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 0, 20, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 40, 0, 10, 0, 0, 0, 40, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle)};

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {obstacleOne}, 0.1));
}

TEST_F(DrivesWithSlightlyHigherSpeedPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, obstacleOne));
}

TEST_F(DrivesWithSlightlyHigherSpeedPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle, obstacleOne), std::runtime_error);
}

TEST_F(DrivesWithSlightlyHigherSpeedPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle, obstacleOne), std::runtime_error);
}

TEST_F(DrivesWithSlightlyHigherSpeedPredicateTest, SetBasedPrediction) {
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

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, ego, {"50.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs1, ego, {"50.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, obs1, ego, {"50.0"}, true));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, obs1, {"50.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, ego, obs1, {"50.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, ego, obs1, {"50.0"}, true));
}
