#include "test_in_intersection_conflict_area_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/geometry/geometric_operations.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void InIntersectionConflictAreaPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 1009.5, 950, 10, 0, 1.5708);
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 1009.5, 995, 10, 0, 1.5708);

    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 1009.5, 990, 10, 0, 1.5708);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 1009.5, 999, 10, 0, 1.5708);

    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 1009.5, 995, 10, 0, 1.5708);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 1007.5, 1007.5, 10, 0, 3.14);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle)};
    state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
    };

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    obstacleOne = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    setUpIncoming();

    auto roadNetwork = std::make_shared<RoadNetwork>(
        RoadNetwork(lanelets, SupportedTrafficSignCountry::GERMANY, {}, {}, {intersection1}));
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(InIntersectionConflictAreaPredicateTest, TestScenario1) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() +
                                  "/predicates/DEU_TestTurnLeft-1/DEU_TestTurnLeft-1_2_T-1.pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne, planningProblemsOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world{std::make_shared<World>(
        World("testWorld", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne))};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0))); //
    //   depends on selected reference since it is not clear whether vehicle turns

    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(
        44, world, obstaclesScenarioOne.at(1),
        obstaclesScenarioOne.at(0))); // depends on selected reference since it is not clear whether
    //        vehicle turns

    EXPECT_FALSE(pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
}

TEST_F(InIntersectionConflictAreaPredicateTest, TestScenario2) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() +
                                  "/predicates/DEU_TestRightBeforeLeft-1/DEU_TestRightBeforeLeft-1_2_T-1.pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne, planningProblemsOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world{std::make_shared<World>(
        World("testWorld", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne))};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(34, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(37, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(37, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_TRUE(pred.booleanEvaluation(39, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(39, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_TRUE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_TRUE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(50, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
}

TEST_F(InIntersectionConflictAreaPredicateTest, TestScenario3) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() +
                                  "/predicates/DEU_TestTurnLeft-1/DEU_TestTurnLeft-1_8_T-1.pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne, planningProblemsOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world{std::make_shared<World>(
        World("testWorld", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne))};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_TRUE(pred.booleanEvaluation(27, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(27, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(37, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(37, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(38, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(38, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(42, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));

    EXPECT_FALSE(pred.booleanEvaluation(49, world, obstaclesScenarioOne.at(0), obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(49, world, obstaclesScenarioOne.at(1), obstaclesScenarioOne.at(0)));
}

TEST_F(InIntersectionConflictAreaPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(InIntersectionConflictAreaPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(InIntersectionConflictAreaPredicateTest, SetBasedPrediction) {
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

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(20, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(29, world, obs1, ego, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(38, world, obs1, ego, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, obs1, ego, {}, true));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs1, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(20, world, ego, obs1, {}, true));
    // last three test cases true because we cannot distinguish whether right turn lanelet is not part of set-based
    // predictions ref lane
    EXPECT_TRUE(pred.booleanEvaluation(29, world, ego, obs1, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(38, world, ego, obs1, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, ego, obs1, {}, true));
}
