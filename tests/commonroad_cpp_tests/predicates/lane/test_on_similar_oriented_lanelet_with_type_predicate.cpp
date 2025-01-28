#include "test_on_similar_oriented_lanelet_with_type_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void OnSimilarOrientedLaneletWithTypePredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 2, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 4, 10, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 6, 10, 0, 0, 0, 30, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 40, 8, 10, 0, 0, 0, 40, 0);

    state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
    };

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, BooleanEvaluationOnShoulder) {
    initializeTestData("SHOULDER", "interstate");
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, BooleanEvaluationOnSUrbanRoad) {
    initializeTestData("URBAN", "INTERSECTION");
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, BooleanEvaluationOnMainCarriageWay) {
    initializeTestData("MAINCARRIAGEWAY", "interstate");
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, BooleanEvaluationOnExitRamp) {
    initializeTestData("EXITRAMP", "interstate");
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, BooleanEvaluationOnAccessRamp) {
    initializeTestData("ACCESSRAMP", "interstate");
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

void OnSimilarOrientedLaneletWithTypePredicateTest::initializeTestData(const std::string &laneletType1,
                                                                       const std::string &laneletType2) {
    auto roadNetwork{
        utils_predicate_test::create_road_network({lanelet_operations::matchStringToLaneletType(laneletType1),
                                                   lanelet_operations::matchStringToLaneletType(laneletType2)})};
    this->world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {this->egoVehicle}, {}, 0.1));
    opt = {laneletType1, laneletType2};
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, BooleanEvaluationIntersection) {
    initializeTestData("INTERSECTION", "URBAN");
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() +
                               "/predicates/DEU_TrafficLightTest-1/DEU_TrafficLightTest-1_1_T-1.pb"};
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 26.5, -7.5, 0, 0, M_PI / 2, 0, 15, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 26., 3, 0, 0, M_PI, 0, 25, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(0, 26.5, -3.5, 0, 0, M_PI / 2, 0, 20, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(0, 7.0, 3, 0, 0, M_PI, 0, 43, 0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne)};

    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    std::shared_ptr<Obstacle> obstacleOne{
        std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2))};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork,
                                          std::vector<std::shared_ptr<Obstacle>>{egoVehicle, obstacleOne}, {},
                                          timeStepSize));
    EXPECT_FALSE(
        pred.booleanEvaluation(0, world, egoVehicle, {}, opt)); // in front of intersection / completely on incoming
    EXPECT_TRUE(
        pred.booleanEvaluation(1, world, egoVehicle, {}, opt)); // standing on stop line -> partially in intersection
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {}, opt));  // inside intersection
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, {}, opt)); // left intersection
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(OnSimilarOrientedLaneletWithTypePredicateTest, SetBasedPredictionIntersection) {
    std::string scenarioName = "ZAM_TestTurnLeft-2_1_S-2";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto obstacles{scenarioXml.obstacles};
    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, obstacles, {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(1001)};

    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, {}, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, ego, {}, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, obs1, {}, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, obs1, {}, {"straight"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, {}, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, ego, {}, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, {}, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, obs1, {}, {"right"}, true));
}
