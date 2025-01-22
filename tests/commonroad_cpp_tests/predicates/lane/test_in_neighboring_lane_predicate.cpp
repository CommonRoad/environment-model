#include "test_in_neighboring_lane_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestInNeighboringLanePredicate::SetUp() {
    std::string pathToTestFile =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_ThreeLanes-1/DEU_ThreeLanes-1_1_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::string pathToTestFileOncoming =
        TestUtils::getTestScenarioDirectory() +
        "/predicates/DEU_TwoLanesWithOppositeDirections-1/DEU_TwoLanesWithOppositeDirections-1_1_T-1.pb";
    const auto &[obstaclesOncoming, roadNetworkOncoming, timeStepSizeOncoming, planningProblemsOncoming] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncoming);

    std::shared_ptr<State> stateZeroObstacleEgo = std::make_shared<State>(0, 110, 2, 2, 0, 0, 0, 110, 0);
    std::shared_ptr<State> stateOneObstacleEgo = std::make_shared<State>(1, 112, 2, 2, 0, 0, 0, 112, 0);
    std::shared_ptr<State> stateTwoObstacleEgo = std::make_shared<State>(2, 114, 2, 2, 0, 0, 0, 114, 0);
    std::shared_ptr<State> stateThreeObstacleEgo = std::make_shared<State>(3, 116, 2, 2, 0, 0, 0, 116, 0);
    std::shared_ptr<State> stateFourObstacleEgo = std::make_shared<State>(4, 118, 2, 2, 0, 0, 0, 118, 0);
    std::shared_ptr<State> stateFiveObstacleEgo = std::make_shared<State>(5, 120, 2, 2, 0, 0, 0, 120, 0);
    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleEgo)};
    obstacleEgo = std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleEgo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 115, 2, 1, 0, 0, 0, 115, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 116, 2, 1, 0, 0, 0, 116.5, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 117, 2, 1, 0, 0, 0, 118, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 118, 2, 1, 0, 0, 0, 119.5, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 119, 2, 1, 0, 0, 0, 121, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 120, 2, 1, 0, 0, 0, 122.5, 0);
    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};
    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 60, 5, 2.5, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 62.5, 5, 2.5, 0, 0, 0, 62.5, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 65, 5, 2.5, 0, 0, 0, 65, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 67.5, 5, 2.5, 0, 0, 0, 67.5, 0);
    std::shared_ptr<State> stateFourObstacleTwo = std::make_shared<State>(4, 70, 5, 2.5, 0, 0, 0, 70, 0);
    std::shared_ptr<State> stateFiveObstacleTwo = std::make_shared<State>(5, 72.5, 5, 2.5, 0, 0, 0, 72.5, 0);
    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleTwo)};
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 105, 8.5, 15, 0, 0, 0, 105, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 120, 8.5, 15, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 135, 5, 15, 0, 0, 0, 135, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 150, 5, 15, 0, 0, 0, 150, 0);
    std::shared_ptr<State> stateFourObstacleThree = std::make_shared<State>(4, 165, 5, 15, 0, 0, 0, 165, 0);
    std::shared_ptr<State> stateFiveObstacleThree = std::make_shared<State>(5, 180, 5, 15, 0, 0, 0, 180, 0);
    state_map_t trajectoryPredictionObstacleThree{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleThree)};
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));

    std::shared_ptr<State> stateZeroObstacleFour = std::make_shared<State>(0, 60, 8.5, 2.5, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateOneObstacleFour = std::make_shared<State>(1, 62.5, 8.5, 2.5, 0, 0, 0, 62.5, 0);
    std::shared_ptr<State> stateTwoObstacleFour = std::make_shared<State>(2, 65, 8.5, 2.5, 0, 0, 0, 65, 0);
    std::shared_ptr<State> stateThreeObstacleFour = std::make_shared<State>(3, 67.5, 8.5, 2.5, 0, 0, 0, 67.5, 0);
    std::shared_ptr<State> stateFourObstacleFour = std::make_shared<State>(4, 70, 8.5, 2.5, 0, 0, 0, 70, 0);
    std::shared_ptr<State> stateFiveObstacleFour = std::make_shared<State>(5, 72.5, 8.5, 2.5, 0, 0, 0, 72.5, 0);
    state_map_t trajectoryPredictionObstacleFour{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFour)};
    obstacleFour =
        std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, stateZeroObstacleFour, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));

    std::shared_ptr<State> stateZeroObstacleFive = std::make_shared<State>(0, 85, 3, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleFive = std::make_shared<State>(1, 80, 3, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleFive = std::make_shared<State>(2, 75, 3, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleFive = std::make_shared<State>(3, 70, 3, 5, 0, 0);
    std::shared_ptr<State> stateFourObstacleFive = std::make_shared<State>(4, 65, 3, 5, 0, 0);
    std::shared_ptr<State> stateFiveObstacleFive = std::make_shared<State>(5, 60, 3, 5, 0, 0);
    state_map_t trajectoryPredictionObstacleFive{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFive)};
    obstacleFive =
        std::make_shared<Obstacle>(Obstacle(5, ObstacleRole::DYNAMIC, stateZeroObstacleFive, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFive, 5, 2));

    std::shared_ptr<State> stateZeroObstacleSix = std::make_shared<State>(0, 10, 0, 2, 0, 0);
    std::shared_ptr<State> stateOneObstacleSix = std::make_shared<State>(1, 12, 0, 2, 0, 0);
    std::shared_ptr<State> stateTwoObstacleSix = std::make_shared<State>(2, 14, 0, 2, 0, 0);
    std::shared_ptr<State> stateThreeObstacleSix = std::make_shared<State>(3, 16, 0, 2, 0, 0);
    std::shared_ptr<State> stateFourObstacleSix = std::make_shared<State>(4, 18, 0, 2, 0, 0);
    std::shared_ptr<State> stateFiveObstacleSix = std::make_shared<State>(5, 20, 0, 2, 0, 0);
    state_map_t trajectoryPredictionObstacleSix{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleSix)};
    obstacleSix = std::make_shared<Obstacle>(Obstacle(6, ObstacleRole::DYNAMIC, stateZeroObstacleSix, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleSix, 5, 2));

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo},
                                          {obstacleOne, obstacleTwo, obstacleThree, obstacleFour}, 0.1));

    worldOncoming =
        std::make_shared<World>(World("testWorld", 0, roadNetworkOncoming, {obstacleSix}, {obstacleFive}, 0.1));
}

TEST_F(TestInNeighboringLanePredicate, BooleanEvaluationObjectsOncoming) {
    // obstacleFive on oncoming lane of obstacleSix
    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleSix, obstacleFive, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleSix, obstacleFive, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleSix, obstacleFive, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleSix, obstacleFive, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldOncoming, obstacleSix, obstacleFive, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(5, worldOncoming, obstacleSix, obstacleFive, {"left"}));

    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleFive, obstacleSix, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleFive, obstacleSix, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleFive, obstacleSix, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleFive, obstacleSix, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldOncoming, obstacleFive, obstacleSix, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(5, worldOncoming, obstacleFive, obstacleSix, {"left"}));
}

TEST_F(TestInNeighboringLanePredicate, BooleanEvaluationObjectsMultilane) {
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    // obstacleTwo on middle lane
    // obstacleThree on left lane changing to middle lane (at timestep 2)
    // obstacleFour on left lane
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleOne, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleTwo, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleThree, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleFour, {"left"}));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleThree, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo, obstacleThree, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleEgo, obstacleThree, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleEgo, obstacleThree, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleEgo, obstacleThree, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleEgo, obstacleThree, {"left"}));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleTwo, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleEgo, obstacleTwo, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleEgo, obstacleTwo, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleEgo, obstacleTwo, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleEgo, obstacleTwo, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleEgo, obstacleTwo, {"left"}));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleTwo, obstacleThree, {"left"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleTwo, obstacleThree, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleThree, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleTwo, obstacleThree, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleTwo, obstacleThree, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleTwo, obstacleThree, {"left"}));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleFour, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleOne, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleEgo, {"left"}));

    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleThree, obstacleEgo, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleThree, obstacleEgo, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleThree, obstacleTwo, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleThree, obstacleTwo, {"left"}));

    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleFour, obstacleOne, {"left"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleFour, {"left"}));
}

TEST_F(TestInNeighboringLanePredicate, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(TestInNeighboringLanePredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

void TestInNeighboringLanePredicate::SetUpRightSide() {
    std::string pathToTestFile =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_ThreeLanes-1/DEU_ThreeLanes-1_1_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::string pathToTestFileOncoming =
        TestUtils::getTestScenarioDirectory() +
        "/predicates/DEU_TwoLanesWithOppositeDirections-1/DEU_TwoLanesWithOppositeDirections-1_1_T-1.pb";
    const auto &[obstaclesOncoming, roadNetworkOncoming, timeStepSizeOncoming, planningProblemsOncoming] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncoming);

    std::shared_ptr<State> stateZeroObstacleEgo = std::make_shared<State>(0, 110, 2, 2, 0, 0, 0, 110, 0);
    std::shared_ptr<State> stateOneObstacleEgo = std::make_shared<State>(1, 112, 2, 2, 0, 0, 0, 112, 0);
    std::shared_ptr<State> stateTwoObstacleEgo = std::make_shared<State>(2, 114, 2, 2, 0, 0, 0, 114, 0);
    std::shared_ptr<State> stateThreeObstacleEgo = std::make_shared<State>(3, 116, 2, 2, 0, 0, 0, 116, 0);
    std::shared_ptr<State> stateFourObstacleEgo = std::make_shared<State>(4, 118, 2, 2, 0, 0, 0, 118, 0);
    std::shared_ptr<State> stateFiveObstacleEgo = std::make_shared<State>(5, 120, 2, 2, 0, 0, 0, 120, 0);
    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleEgo),
                                               std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleEgo)};
    obstacleEgo = std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleEgo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 115, 2, 1, 0, 0, 0, 115, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 116, 2, 1, 0, 0, 0, 116.5, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 117, 2, 1, 0, 0, 0, 118, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 118, 2, 1, 0, 0, 0, 119.5, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 119, 2, 1, 0, 0, 0, 121, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 120, 2, 1, 0, 0, 0, 122.5, 0);
    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};
    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 60, 5, 2.5, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 62.5, 5, 2.5, 0, 0, 0, 62.5, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 65, 5, 2.5, 0, 0, 0, 65, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 67.5, 5, 2.5, 0, 0, 0, 67.5, 0);
    std::shared_ptr<State> stateFourObstacleTwo = std::make_shared<State>(4, 70, 5, 2.5, 0, 0, 0, 70, 0);
    std::shared_ptr<State> stateFiveObstacleTwo = std::make_shared<State>(5, 72.5, 5, 2.5, 0, 0, 0, 72.5, 0);
    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleTwo)};
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 105, 8.5, 15, 0, 0, 0, 105, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 120, 8.5, 15, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 135, 5, 15, 0, 0, 0, 135, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 150, 5, 15, 0, 0, 0, 150, 0);
    std::shared_ptr<State> stateFourObstacleThree = std::make_shared<State>(4, 165, 5, 15, 0, 0, 0, 165, 0);
    std::shared_ptr<State> stateFiveObstacleThree = std::make_shared<State>(5, 180, 5, 15, 0, 0, 0, 180, 0);
    state_map_t trajectoryPredictionObstacleThree{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleThree),
                                                  std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleThree)};
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));

    std::shared_ptr<State> stateZeroObstacleFour = std::make_shared<State>(0, 60, 8.5, 2.5, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateOneObstacleFour = std::make_shared<State>(1, 62.5, 8.5, 2.5, 0, 0, 0, 62.5, 0);
    std::shared_ptr<State> stateTwoObstacleFour = std::make_shared<State>(2, 65, 8.5, 2.5, 0, 0, 0, 65, 0);
    std::shared_ptr<State> stateThreeObstacleFour = std::make_shared<State>(3, 67.5, 8.5, 2.5, 0, 0, 0, 67.5, 0);
    std::shared_ptr<State> stateFourObstacleFour = std::make_shared<State>(4, 70, 8.5, 2.5, 0, 0, 0, 70, 0);
    std::shared_ptr<State> stateFiveObstacleFour = std::make_shared<State>(5, 72.5, 8.5, 2.5, 0, 0, 0, 72.5, 0);
    state_map_t trajectoryPredictionObstacleFour{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleFour),
                                                 std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFour)};
    obstacleFour =
        std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, stateZeroObstacleFour, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));

    std::shared_ptr<State> stateZeroObstacleFive = std::make_shared<State>(0, 85, 3, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleFive = std::make_shared<State>(1, 80, 3, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleFive = std::make_shared<State>(2, 75, 3, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleFive = std::make_shared<State>(3, 70, 3, 5, 0, 0);
    std::shared_ptr<State> stateFourObstacleFive = std::make_shared<State>(4, 65, 3, 5, 0, 0);
    std::shared_ptr<State> stateFiveObstacleFive = std::make_shared<State>(5, 60, 3, 5, 0, 0);
    state_map_t trajectoryPredictionObstacleFive{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleFive),
                                                 std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFive)};
    obstacleFive =
        std::make_shared<Obstacle>(Obstacle(5, ObstacleRole::DYNAMIC, stateZeroObstacleFive, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFive, 5, 2));

    std::shared_ptr<State> stateZeroObstacleSix = std::make_shared<State>(0, 10, 0, 2, 0, 0);
    std::shared_ptr<State> stateOneObstacleSix = std::make_shared<State>(1, 12, 0, 2, 0, 0);
    std::shared_ptr<State> stateTwoObstacleSix = std::make_shared<State>(2, 14, 0, 2, 0, 0);
    std::shared_ptr<State> stateThreeObstacleSix = std::make_shared<State>(3, 16, 0, 2, 0, 0);
    std::shared_ptr<State> stateFourObstacleSix = std::make_shared<State>(4, 18, 0, 2, 0, 0);
    std::shared_ptr<State> stateFiveObstacleSix = std::make_shared<State>(5, 20, 0, 2, 0, 0);
    state_map_t trajectoryPredictionObstacleSix{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleSix),
                                                std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleSix)};
    obstacleSix = std::make_shared<Obstacle>(Obstacle(6, ObstacleRole::DYNAMIC, stateZeroObstacleSix, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleSix, 5, 2));

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {obstacleEgo},
                                          {obstacleOne, obstacleTwo, obstacleThree, obstacleFour}, 0.1));

    worldOncoming =
        std::make_shared<World>(World("testWorld", 0, roadNetworkOncoming, {obstacleSix}, {obstacleFive}, 0.1));
}

TEST_F(TestInNeighboringLanePredicate, BooleanEvaluationObjectsOncomingRightSide) {
    // obstacleFive on oncoming lane of obstacleSix
    SetUpRightSide();
    EXPECT_FALSE(pred.booleanEvaluation(0, worldOncoming, obstacleSix, obstacleFive, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOncoming, obstacleSix, obstacleFive, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOncoming, obstacleSix, obstacleFive, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldOncoming, obstacleSix, obstacleFive, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleSix, obstacleFive, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, worldOncoming, obstacleSix, obstacleFive, {"right"}));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldOncoming, obstacleFive, obstacleSix, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOncoming, obstacleFive, obstacleSix, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOncoming, obstacleFive, obstacleSix, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldOncoming, obstacleFive, obstacleSix, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleFive, obstacleSix, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, worldOncoming, obstacleFive, obstacleSix, {"right"}));
}

TEST_F(TestInNeighboringLanePredicate, BooleanEvaluationObjectsMultilaneRightSide) {
    void SetUpRightSide();
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    // obstacleTwo on middle lane
    // obstacleThree on left lane changing to middle lane (at timestep 2)
    // obstacleFour on left lane
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleOne, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleTwo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleFour, {"right"}));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleEgo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleEgo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleEgo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleEgo, obstacleThree, {"right"}));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleTwo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo, obstacleTwo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleEgo, obstacleTwo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleEgo, obstacleTwo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleEgo, obstacleTwo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleEgo, obstacleTwo, {"right"}));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleTwo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleTwo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleTwo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleTwo, obstacleThree, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleTwo, obstacleThree, {"right"}));

    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleFour, {"right"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleOne, {"right"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleEgo, {"right"}));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleThree, obstacleEgo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleThree, obstacleEgo, {"right"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleThree, obstacleTwo, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleThree, obstacleTwo, {"right"}));

    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleFour, obstacleOne, {"right"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleFour, {"right"}));
}

TEST_F(TestInNeighboringLanePredicate, SetBasedPrediction) {
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

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs1, {"left"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs2, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, obs3, {"left"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, ego, obs1, {"left"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, ego, obs2, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, ego, obs3, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(25, world, ego, obs1, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(25, world, ego, obs2, {"left"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(25, world, ego, obs3, {"left"}, true));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs1, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs2, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs3, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, ego, obs1, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, ego, obs2, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, ego, obs3, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, ego, obs1, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, ego, obs2, {"right"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, ego, obs3, {"right"}, true));
}
