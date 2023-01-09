#include "test_safe_distance_gap_right_violated_predicate.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/obstacle/state.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestSafeDistanceGapRightViolatedPredicate::SetUp() {
    pathToTestFile = TestUtils::getTestScenarioDirectory() + "/predicates/DEU_three_lanes.xml";
    pathToTestFileOvertaking =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_test_safe_distance_to_right_vehicle_violated.xml";

    std::shared_ptr<State> stateZeroObstacleEgo = std::make_shared<State>(0, 110, 2, 2, 0, 0, 0, 110, 0);
    std::shared_ptr<State> stateOneObstacleEgo = std::make_shared<State>(1, 112, 2, 2, 0, 0, 0, 112, 0);
    std::shared_ptr<State> stateTwoObstacleEgo = std::make_shared<State>(2, 114, 2, 2, 0, 0, 0, 114, 0);
    std::shared_ptr<State> stateThreeObstacleEgo = std::make_shared<State>(3, 116, 2, 2, 0, 0, 0, 116, 0);
    std::shared_ptr<State> stateFourObstacleEgo = std::make_shared<State>(4, 118, 2, 2, 0, 0, 0, 118, 0);
    std::shared_ptr<State> stateFiveObstacleEgo = std::make_shared<State>(5, 120, 2, 2, 0, 0, 0, 120, 0);
    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleEgo),
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
    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
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
    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleTwo)};
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 90, 8.5, 15, 0, 0, 0, 90, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 105, 8.5, 15, 0, 0, 0, 105, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 120, 5, 15, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 135, 5, 15, 0, 0, 0, 135, 0);
    std::shared_ptr<State> stateFourObstacleThree = std::make_shared<State>(4, 150, 5, 15, 0, 0, 0, 150, 0);
    std::shared_ptr<State> stateFiveObstacleThree = std::make_shared<State>(5, 165, 5, 15, 0, 0, 0, 165, 0);
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

    std::shared_ptr<State> stateZeroObstacleFour = std::make_shared<State>(0, 60, 8.5, 2.5, 0, 0, 0, 60, 0);
    std::shared_ptr<State> stateOneObstacleFour = std::make_shared<State>(1, 62.5, 8.5, 2.5, 0, 0, 0, 62.5, 0);
    std::shared_ptr<State> stateTwoObstacleFour = std::make_shared<State>(2, 65, 8.5, 2.5, 0, 0, 0, 65, 0);
    std::shared_ptr<State> stateThreeObstacleFour = std::make_shared<State>(3, 67.5, 8.5, 2.5, 0, 0, 0, 67.5, 0);
    std::shared_ptr<State> stateFourObstacleFour = std::make_shared<State>(4, 70, 8.5, 2.5, 0, 0, 0, 70, 0);
    std::shared_ptr<State> stateFiveObstacleFour = std::make_shared<State>(5, 72.5, 8.5, 2.5, 0, 0, 0, 72.5, 0);
    Obstacle::state_map_t trajectoryPredictionObstacleFour{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFour),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFour),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFour),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFour),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleFour),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFour)};
    obstacleFour =
        std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, stateZeroObstacleFour, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));

    std::shared_ptr<State> stateZeroObstacleFive = std::make_shared<State>(0, 135, 5, 15, 0, 0, 0, 135, 0);
    std::shared_ptr<State> stateOneObstacleFive = std::make_shared<State>(1, 120, 5, 15, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateTwoObstacleFive = std::make_shared<State>(2, 105, 5, 15, 0, 0, 0, 105, 0);
    std::shared_ptr<State> stateThreeObstacleFive = std::make_shared<State>(3, 90, 5, 15, 0, 0, 0, 90, 0);
    std::shared_ptr<State> stateFourObstacleFive = std::make_shared<State>(4, 75, 5, 15, 0, 0, 0, 75, 0);
    std::shared_ptr<State> stateFiveObstacleFive = std::make_shared<State>(5, 60, 5, 15, 0, 0, 0, 60, 0);
    Obstacle::state_map_t trajectoryPredictionObstacleSix{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFive),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFive),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFive),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFive),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleFive),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFive)};
    obstacleFive =
        std::make_shared<Obstacle>(Obstacle(5, ObstacleRole::DYNAMIC, stateZeroObstacleFive, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleSix, 5, 2));
}

TEST_F(TestSafeDistanceGapRightViolatedPredicate, BooleanEvaluationObjectsMultilane) {
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    // obstacleTwo on middle lane
    // obstacleThree on left lane changing to middle lane (at timestep 2)
    // obstacleFour on left lane
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    world = std::make_shared<World>(
        World(0, roadNetwork, {obstacleEgo}, {obstacleOne, obstacleTwo, obstacleThree, obstacleFour}, 0.1));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleEgo));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleEgo));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleOne));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleTwo));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleThree));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleThree));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleThree));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacleThree));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleThree));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleFour));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleFour));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleFour));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleFour));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleFour));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleFour));
}

TEST_F(TestSafeDistanceGapRightViolatedPredicate, BooleanEvaluationObjectsMultilaneOvertaking) {
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFileOvertaking);
    world = std::make_shared<World>(World(0, roadNetwork, {obstacles[2]}, {obstacles[1], obstacles[0]}, 0.1));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(6, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(7, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(8, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(9, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(10, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(11, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(12, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(13, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(14, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(15, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(16, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(17, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(18, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(19, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(20, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(21, world, obstacles[2]));
    EXPECT_TRUE(pred.booleanEvaluation(22, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(23, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(24, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(26, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(27, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(28, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(29, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(30, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(32, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(33, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(34, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(35, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(36, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(37, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(38, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(39, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(41, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(42, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(43, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(45, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(46, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(47, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(48, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(49, world, obstacles[2]));
    EXPECT_FALSE(pred.booleanEvaluation(50, world, obstacles[2]));
}

TEST_F(TestSafeDistanceGapRightViolatedPredicate, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleEgo), std::runtime_error);
}

TEST_F(TestSafeDistanceGapRightViolatedPredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleEgo), std::runtime_error);
}
