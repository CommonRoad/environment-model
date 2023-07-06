#include "test_oncoming_vehicle_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/obstacle/state.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestOncomingVehiclePredicate::SetUp() {
    pathToTestFileOncoming = TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TwoLanesWithOppositeDirections-1/DEU_TwoLanesWithOppositeDirections-1_1_T-1.pb";
    pathToTestFile = TestUtils::getTestScenarioDirectory() + "/predicates/DEU_ThreeLanes-1/DEU_ThreeLanes-1_1_T-1.pb";

    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    // obstacleTwo on middle lane
    // obstacleThree on left lane changing to middle lane (at timestep 2)
    // obstacleFour on left lane
    std::shared_ptr<State> stateZeroObstacleEgo = std::make_shared<State>(0, 10, 0, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleEgo = std::make_shared<State>(1, 15, 0, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleEgo = std::make_shared<State>(2, 20, 0, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleEgo = std::make_shared<State>(3, 25, 0, 5, 0, 0);
    std::shared_ptr<State> stateFourObstacleEgo = std::make_shared<State>(4, 30, 0, 5, 0, 0);
    std::shared_ptr<State> stateFiveObstacleEgo = std::make_shared<State>(5, 35, 0, 5, 0, 0);
    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleEgo),
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
    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};
    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 85, 3, 15, 0, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 70, 3, 15, 0, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 55, 3, 15, 0, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 40, 3, 15, 0, 0);
    std::shared_ptr<State> stateFourObstacleTwo = std::make_shared<State>(4, 25, 3, 15, 0, 0);
    std::shared_ptr<State> stateFiveObstacleTwo = std::make_shared<State>(5, 5, 3, 15, 0, 0);
    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleTwo)};
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 40, 3, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 50, 3, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 60, 3, 10, 0, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 70, 3, 10, 0, 0);
    std::shared_ptr<State> stateFourObstacleThree = std::make_shared<State>(4, 80, 3, 10, 0, 0);
    std::shared_ptr<State> stateFiveObstacleThree = std::make_shared<State>(5, 90, 3, 10, 0, 0);
    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleThree),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleThree),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleThree)};
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
}

TEST_F(TestOncomingVehiclePredicate, BooleanEvaluationObjectsMultilane) {
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    std::shared_ptr<World> world =
        std::make_shared<World>(World(0, roadNetwork, {obstacleEgo}, {obstacleOne, obstacleThree}, 0.1));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleEgo));
}

TEST_F(TestOncomingVehiclePredicate, BooleanEvaluationObjectsNoOncomingTraffic) {
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    const auto &[obstaclesOncoming, roadNetworkOncoming, timeStepSizeOncoming] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncoming);
    std::shared_ptr<World> worldOncoming =
        std::make_shared<World>(World(0, roadNetworkOncoming, {obstacleEgo}, {obstacleOne}, 0.1));
    EXPECT_FALSE(pred.booleanEvaluation(0, worldOncoming, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOncoming, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOncoming, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldOncoming, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleEgo));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldOncoming, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOncoming, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOncoming, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(3, worldOncoming, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleOne));
}

TEST_F(TestOncomingVehiclePredicate, BooleanEvaluationObjectsOncomingTraffic) {
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    // obstacleTwo on oncoming lane
    const auto &[obstaclesOncoming, roadNetworkOncoming, timeStepSizeOncoming] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncoming);
    std::shared_ptr<World> worldOncoming =
        std::make_shared<World>(World(0, roadNetworkOncoming, {obstacleEgo}, {obstacleTwo}, 0.1));
    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleEgo));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleEgo));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleEgo));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(5, worldOncoming, obstacleEgo));
    //
    worldOncoming =
        std::make_shared<World>(World(0, roadNetworkOncoming, {obstacleTwo}, {obstacleEgo, obstacleOne}, 0.1));
    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldOncoming, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(5, worldOncoming, obstacleTwo));
}

TEST_F(TestOncomingVehiclePredicate, RobustEvaluation) {
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    std::shared_ptr<World> world =
        std::make_shared<World>(World(0, roadNetwork, {obstacleEgo}, {obstacleOne, obstacleThree}, 0.1));
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(TestOncomingVehiclePredicate, ConstraintEvaluation) {
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    std::shared_ptr<World> world =
        std::make_shared<World>(World(0, roadNetwork, {obstacleEgo}, {obstacleOne, obstacleThree}, 0.1));
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}