#include "test_neighboring_lane_opp_driving_dir_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestNeighboringLaneOppDrivingDirPredicate::SetUp() {
    pathToTestFileOncoming =
        TestUtils::getTestScenarioDirectory() +
        "/predicates/DEU_TwoLanesWithOppositeDirections-1/DEU_TwoLanesWithOppositeDirections-1_1_T-1.pb";
    pathToTestFile = TestUtils::getTestScenarioDirectory() + "/predicates/DEU_ThreeLanes-1/DEU_ThreeLanes-1_1_T-1.pb";
    initObstacles();
}
void TestNeighboringLaneOppDrivingDirPredicate::initObstacles() {
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    std::shared_ptr<State> stateZeroObstacleEgo = std::make_shared<State>(0, 10, 0, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleEgo = std::make_shared<State>(1, 15, 0, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleEgo = std::make_shared<State>(2, 20, 0, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleEgo = std::make_shared<State>(3, 25, 0, 5, 0, 0);
    std::shared_ptr<State> stateFourObstacleEgo = std::make_shared<State>(4, 30, 0, 5, 0, 0);
    std::shared_ptr<State> stateFiveObstacleEgo = std::make_shared<State>(5, 35, 0, 5, 0, 0);
    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleEgo)};
    obstacleEgo = std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleEgo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 35, 0, 1, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 36, 0, 1, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 37, 0, 1, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 38, 0, 1, 0, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 39, 0, 1, 0, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 40, 0, 1, 0, 0);
    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};
    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
}

TEST_F(TestNeighboringLaneOppDrivingDirPredicate, BooleanEvaluationObjectsMultilane) {
    initObstacles();
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo}, {obstacleOne}, 0.1));

    std::vector<std::string> opt;
    opt = {"left"};
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleEgo, {}, opt));

    opt = {"right"};
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleEgo, {}, opt));
}

TEST_F(TestNeighboringLaneOppDrivingDirPredicate, BooleanEvaluationObjectsNoOncomingTraffic) {
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    initObstacles();
    const auto &[obstaclesOncoming, roadNetworkOncoming, timeStepSizeOncoming, planningProblemsOncoming] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncoming);
    std::shared_ptr<World> worldOncoming =
        std::make_shared<World>(World("testWorld", 0, roadNetworkOncoming, {obstacleEgo}, {obstacleOne}, 0.1));

    std::vector<std::string> opt;
    opt = {"left"};
    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldOncoming, obstacleEgo, {}, opt));

    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleOne, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleOne, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleOne, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleOne, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldOncoming, obstacleOne, {}, opt));

    opt = {"right"};
    EXPECT_FALSE(pred.booleanEvaluation(0, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldOncoming, obstacleEgo, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleEgo, {}, opt));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldOncoming, obstacleOne, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOncoming, obstacleOne, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOncoming, obstacleOne, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldOncoming, obstacleOne, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleOne, {}, opt));
}

TEST_F(TestNeighboringLaneOppDrivingDirPredicate, RobustEvaluation) {
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo}, {obstacleOne}, 0.1));
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(TestNeighboringLaneOppDrivingDirPredicate, ConstraintEvaluation) {
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    std::shared_ptr<World> world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo}, {obstacleOne}, 0.1));
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
