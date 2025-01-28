#include "test_at_intersection_type_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void AtIntersectionTypePredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() +
                               "/predicates/USA_Test4WayStopIntersection-1_1_T.1.xml"}; // this has STOP signs
    std::string pathToTestFile2{
        TestUtils::getTestScenarioDirectory() +
        "/predicates/DEU_TrafficLightTest-1/DEU_TrafficLightTest-1_1_T-1.pb"}; // this has no STOP signs
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    const auto &[obstacles2, roadNetwork2, timeStepSize2, planningProblems2] =
        InputUtils::getDataFromCommonRoad(pathToTestFile2);

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 25, 3, 10, 0, -M_PI / 2, 0, 25, 3);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 23.5, 15, 10, 0, -M_PI / 2, 0, 23.5, 15);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 28.6, 3.5, 10, 0, M_PI, 0, 10, 3.5);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 43, 3, 10, 0, M_PI, 0, 10, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
    world_2 = std::make_shared<World>(World("testWorld", 0, roadNetwork2, {egoVehicle}, {}, 0.1));
}

TEST_F(AtIntersectionTypePredicateTest, TestFourWayStop) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, {}, {"four_way_stop_intersection"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, {"four_way_stop_intersection"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, {"four_way_stop_intersection"}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle, {}, {"four_way_stop_intersection"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world_2, egoVehicle, {}, {"four_way_stop_intersection"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world_2, egoVehicle, {}, {"four_way_stop_intersection"}));
}

void AtIntersectionTypePredicateTest::SetUpTIntersection() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/T_intersection.xml"};
    std::string pathToTestFile_2{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_Intersection.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    const auto &[obstacles_2, roadNetwork_2, timeStepSize_2, planningProblems_2] =
        InputUtils::getDataFromCommonRoad(pathToTestFile_2);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 17, 0.0, 0, 0, 0, 0, 17, 0.0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 25, 0, 0, 0, 0, 0, 25, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 23, 3, 0, 0, M_PI, 0, 23, 0.0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 40, 3, 0, 0, M_PI, 0, 10, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 23.5, 15, 0, 0, -M_PI / 2, 0, 3, 0.0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 23.5, 30, 0, 0, -M_PI / 2, 0, -3.5, 0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork,
                                          std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {},
                                          timeStepSize));

    world_2 = std::make_shared<World>(World("testWorld", 0, roadNetwork_2,
                                            std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {},
                                            timeStepSize_2));
}

TEST_F(AtIntersectionTypePredicateTest, TestTIntersection) {
    SetUpTIntersection();
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleTwo, {}, {"t_intersection"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, {}, {"t_intersection"}));

    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleTwo, {}, {"t_intersection"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, {}, {"t_intersection"}));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, {}, {"t_intersection"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo, {}, {"t_intersection"}));

    EXPECT_FALSE(pred.booleanEvaluation(2, world_2, obstacleOne, {}, {"t_intersection"})); // not on t-intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world_2, obstacleTwo, {}, {"t_intersection"})); // not on t-intersection
}

void AtIntersectionTypePredicateTest::SetUpUncontrolledIntersection() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() +
                               "/predicates/DEU_Intersection.xml"}; // not an uncontrolled intersection
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::string pathToTestFile_2{TestUtils::getTestScenarioDirectory() + "/predicates/uncontrolled_intersection.xml"};
    const auto &[obstacles_2, roadNetwork_2, timeStepSize_2, planningProblem_2] =
        InputUtils::getDataFromCommonRoad(pathToTestFile_2);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 17.0, 0.0, 0, 0, 0, 0, 17, 0.0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 15, 0, 0, 0, 0, 0, 15, 0.0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 26.5, 0, 0, 0, M_PI / 2, 0, 26, 0.0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 23.5, 0, 0, 0, -M_PI / 2, 0, 26, 0.0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(3, 40.5, 3, 0, 0, M_PI, 0, 9.5, 0.0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(3, 23.5, 15, 0, 0, -M_PI / 2, 0, 10.5, 0.0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, timeStepSize));

    world_2 = std::make_shared<World>(
        World("testWorld", 0, roadNetwork_2, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, timeStepSize));
}

TEST_F(AtIntersectionTypePredicateTest, TestUncontrolledIntersection) {
    SetUpUncontrolledIntersection();
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne, {},
                                        {"uncontrolled_intersection"})); // not at uncontrolled intersection

    EXPECT_TRUE(pred.booleanEvaluation(1, world_2, obstacleOne, {},
                                       {"uncontrolled_intersection"})); // approach uncontrolled intersection

    EXPECT_TRUE(
        pred.booleanEvaluation(2, world_2, obstacleOne, {},
                               {"uncontrolled_intersection"})); // in uncontrolled intersection but does not approach
    EXPECT_TRUE(
        pred.booleanEvaluation(3, world_2, obstacleOne, {},
                               {"uncontrolled_intersection"})); // in uncontrolled intersection but does not approach

    EXPECT_TRUE(pred.booleanEvaluation(4, world_2, obstacleOne, {},
                                       {"uncontrolled_intersection"})); // approach uncontrolled intersection
    EXPECT_TRUE(pred.booleanEvaluation(5, world_2, obstacleOne, {},
                                       {"uncontrolled_intersection"})); // approach uncontrolled intersection
}

TEST_F(AtIntersectionTypePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(AtIntersectionTypePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(AtIntersectionTypePredicateTest, SetBasedPrediction) {
    std::string scenarioName = "USA_Lanker-1_1_S-2";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto obs1{world->findObstacle(1213)};
    auto obs2{world->findObstacle(1219)};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, {}, {"UNKNOWN"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs1, {}, {"UNKNOWN"}, true));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs1, {}, {"FOUR_WAY_INTERSECTION"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs1, {}, {"FOUR_WAY_INTERSECTION"}, true));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs2, {}, {"UNKNOWN"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obs2, {}, {"UNKNOWN"}, true));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs2, {}, {"FOUR_WAY_INTERSECTION"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obs2, {}, {"FOUR_WAY_INTERSECTION"}, true));
}
