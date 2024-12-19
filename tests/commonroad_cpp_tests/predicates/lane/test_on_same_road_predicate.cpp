#include "test_on_same_road_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void TestOnSameRoadPredicate::SetUp() {
    std::string pathToTestFile =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_ThreeLanes-1/DEU_ThreeLanes-1_1_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::string pathToTestFileOncoming =
        TestUtils::getTestScenarioDirectory() +
        "/predicates/DEU_TwoLanesWithOppositeDirections-1/DEU_TwoLanesWithOppositeDirections-1_1_T-1.pb";
    const auto &[obstaclesOncoming, roadNetworkOncoming, timeStepSizeOncoming] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncoming);

    std::shared_ptr<State> stateZeroObstacleEgo = std::make_shared<State>(0, 110, 2, 2, 0, 0, 0, 110, 0);
    std::shared_ptr<State> stateOneObstacleEgo = std::make_shared<State>(1, 112, 2, 2, 0, 0, 0, 112, 0);
    std::shared_ptr<State> stateTwoObstacleEgo = std::make_shared<State>(2, 114, 2, 2, 0, 0, 0, 114, 0);
    std::shared_ptr<State> stateThreeObstacleEgo = std::make_shared<State>(3, 116, 2, 2, 0, 0, 0, 116, 0);
    std::shared_ptr<State> stateFourObstacleEgo = std::make_shared<State>(4, 118, 2, 2, 0, 0, 0, 118, 0);
    std::shared_ptr<State> stateFiveObstacleEgo = std::make_shared<State>(5, 120, 2, 2, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateSixObstacleEgo = std::make_shared<State>(6, 121, 2, 2, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateSevenObstacleEgo = std::make_shared<State>(7, 122, 2, 2, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateEightObstacleEgo = std::make_shared<State>(8, 123, 2, 2, 0, 0, 0, 120, 0);
    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleEgo),
        std::pair<int, std::shared_ptr<State>>(8, stateSevenObstacleEgo)};
    obstacleEgo = std::make_shared<Obstacle>(Obstacle(0, ObstacleRole::DYNAMIC, stateZeroObstacleEgo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 115, 2, 1, 0, 0, 0, 115, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 116, 2, 1, 0, 0, 0, 116.5, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 117, 2, 1, 0, 0, 0, 118, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 118, 2, 1, 0, 0, 0, 119.5, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 119, 2, 1, 0, 0, 0, 121, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(5, 120, 2, 1, 0, 0, 0, 122.5, 0);
    std::shared_ptr<State> stateSixObstacleOne = std::make_shared<State>(6, 199, 2, 1, 0, 0, 0, 122.5, 0);
    std::shared_ptr<State> stateSevenObstacleOne = std::make_shared<State>(7, 201, 2, 1, 0, 0, 0, 122.5, 0);
    std::shared_ptr<State> stateEightObstacleOne = std::make_shared<State>(8, 250, 2, 1, 0, 0, 0, 122.5, 0);
    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleOne),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleOne),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleOne)};
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

    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 105, 8.5, 15, 0, 0, 0, 105, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 120, 8.5, 15, 0, 0, 0, 120, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 135, 5, 15, 0, 0, 0, 135, 0);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 150, 5, 15, 0, 0, 0, 150, 0);
    std::shared_ptr<State> stateFourObstacleThree = std::make_shared<State>(4, 165, 5, 15, 0, 0, 0, 165, 0);
    std::shared_ptr<State> stateFiveObstacleThree = std::make_shared<State>(5, 180, 5, 15, 0, 0, 0, 180, 0);
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

    std::shared_ptr<State> stateZeroObstacleFive = std::make_shared<State>(0, 85, 3, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleFive = std::make_shared<State>(1, 80, 3, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleFive = std::make_shared<State>(2, 75, 3, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleFive = std::make_shared<State>(3, 70, 3, 5, 0, 0);
    std::shared_ptr<State> stateFourObstacleFive = std::make_shared<State>(4, 65, 3, 5, 0, 0);
    std::shared_ptr<State> stateFiveObstacleFive = std::make_shared<State>(5, 60, 3, 5, 0, 0);
    Obstacle::state_map_t trajectoryPredictionObstacleFive{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleFive),
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
    Obstacle::state_map_t trajectoryPredictionObstacleSix{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleSix),
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

TEST_F(TestOnSameRoadPredicate, BooleanEvaluationObjectsOncoming) {
    // obstacleFive on oncoming lane of obstacleSix
    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleSix, obstacleFive));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleSix, obstacleFive));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleSix, obstacleFive));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleSix, obstacleFive));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldOncoming, obstacleSix, obstacleFive));
    EXPECT_TRUE(pred.booleanEvaluation(5, worldOncoming, obstacleSix, obstacleFive));

    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncoming, obstacleFive, obstacleSix));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncoming, obstacleFive, obstacleSix));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncoming, obstacleFive, obstacleSix));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldOncoming, obstacleFive, obstacleSix));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldOncoming, obstacleFive, obstacleSix));
    EXPECT_TRUE(pred.booleanEvaluation(5, worldOncoming, obstacleFive, obstacleSix));
}

TEST_F(TestOnSameRoadPredicate, BooleanEvaluationObjectsMultilane) {
    // obstacleEgo on right lane
    // obstacleOne on right lane in front of obstacleEgo
    // obstacleTwo on middle lane
    // obstacleThree on left lane changing to middle lane (at timestep 2)
    // obstacleFour on left lane
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleFour));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleEgo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleEgo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleEgo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleEgo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleEgo, obstacleThree));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleEgo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleEgo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleEgo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleEgo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleEgo, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleEgo, obstacleTwo));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleTwo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleTwo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleTwo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, obstacleTwo, obstacleThree));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleTwo, obstacleThree));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleFour));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleTwo, obstacleEgo));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleThree, obstacleEgo));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleThree, obstacleEgo));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleThree, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleThree, obstacleTwo));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleFour, obstacleOne));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, obstacleFour));

    // obstacle only partially on road network
    EXPECT_TRUE(pred.booleanEvaluation(6, world, obstacleEgo, obstacleOne));
    // obstacle partially on road network but state not
    EXPECT_FALSE(pred.booleanEvaluation(7, world, obstacleEgo, obstacleOne));
    // obstacle not on road network
    EXPECT_FALSE(pred.booleanEvaluation(8, world, obstacleEgo, obstacleOne));
}

TEST_F(TestOnSameRoadPredicate, BooleanEvaluationDifferentIntersectionIncoming) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() +
                               "/predicates/USA_Test4WayStopIntersection-1_1_T.1.xml"}; // this has STOP signs
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 26.5, -15, 10, 0, M_PI / 2);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 26.5, -5, 10, 0, M_PI / 2);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 23.0, 2.25, 10, 0, (3 * M_PI) / 4);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 17.0, 3.0, 10, 0, M_PI);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 5, 3.0, 10, 0, M_PI);

    std::shared_ptr<State> stateZeroObs1 = std::make_shared<State>(0, 45, 3, 10, 0, M_PI);
    std::shared_ptr<State> stateOneObs1 = std::make_shared<State>(1, 35, 3, 10, 0, M_PI);
    std::shared_ptr<State> stateTwoObs1 = std::make_shared<State>(2, 27, 3, 10, 0, M_PI);
    std::shared_ptr<State> stateThreeObs1 = std::make_shared<State>(3, 23.5, 3.0, 10, 0, M_PI);
    std::shared_ptr<State> stateFourObs1 = std::make_shared<State>(4, 12, 3, 10, 0, M_PI);

    std::shared_ptr<State> stateZeroObs2 = std::make_shared<State>(0, 23.5, 20, 10, 0, -M_PI / 2);
    std::shared_ptr<State> stateOneObs2 = std::make_shared<State>(1, 23.5, 15, 10, 0, -M_PI / 2);
    std::shared_ptr<State> stateTwoObs2 = std::make_shared<State>(2, 25.0, 2.5, 10, 0, -M_PI / 2);
    std::shared_ptr<State> stateThreeObs2 = std::make_shared<State>(3, 23.5, -8, 10, 0, -M_PI / 2);
    std::shared_ptr<State> stateFourObs2 = std::make_shared<State>(4, 23.5, -18, 10, 0, -M_PI / 2);

    std::shared_ptr<State> stateZeroObs3 = std::make_shared<State>(0, 5, 0, 10, 0, 0);
    std::shared_ptr<State> stateOneObs3 = std::make_shared<State>(1, 15, 0, 10, 0, 0);
    std::shared_ptr<State> stateTwoObs3 = std::make_shared<State>(2, 22.5, -0.5, 10, 0, -M_PI / 4);
    std::shared_ptr<State> stateThreeObs3 = std::make_shared<State>(3, 23.5, -10, 10, 0, -M_PI / 2);
    std::shared_ptr<State> stateFourObs3 = std::make_shared<State>(4, 23.5, -15, 10, 0, -M_PI / 2);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle)};

    Obstacle::state_map_t trajectoryPredictionObs1{std::pair<int, std::shared_ptr<State>>(1, stateOneObs1),
                                                   std::pair<int, std::shared_ptr<State>>(2, stateTwoObs1),
                                                   std::pair<int, std::shared_ptr<State>>(3, stateThreeObs1),
                                                   std::pair<int, std::shared_ptr<State>>(4, stateFourObs1)};

    Obstacle::state_map_t trajectoryPredictionObs2{std::pair<int, std::shared_ptr<State>>(1, stateOneObs2),
                                                   std::pair<int, std::shared_ptr<State>>(2, stateTwoObs2),
                                                   std::pair<int, std::shared_ptr<State>>(3, stateThreeObs2),
                                                   std::pair<int, std::shared_ptr<State>>(4, stateFourObs2)};

    Obstacle::state_map_t trajectoryPredictionObs3{std::pair<int, std::shared_ptr<State>>(1, stateOneObs3),
                                                   std::pair<int, std::shared_ptr<State>>(2, stateTwoObs3),
                                                   std::pair<int, std::shared_ptr<State>>(3, stateThreeObs3),
                                                   std::pair<int, std::shared_ptr<State>>(4, stateFourObs3)};

    auto egoVehicle =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car, 50, 10, 3,
                                            -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    auto obs1 = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObs1, ObstacleType::car, 50, 10,
                                                    3, -10, 0.3, trajectoryPredictionObs1, 5, 2));
    auto obs2 = std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObs2, ObstacleType::car, 50, 10,
                                                    3, -10, 0.3, trajectoryPredictionObs2, 5, 2));
    auto obs3 = std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, stateZeroObs3, ObstacleType::car, 50, 10,
                                                    3, -10, 0.3, trajectoryPredictionObs3, 5, 2));

    auto worldFourWay =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {obs1, obs2, obs3}, 0.1));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldFourWay, egoVehicle, obs1));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldFourWay, egoVehicle, obs1));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldFourWay, egoVehicle,
                                        obs1)); // the state must be in the road; otherwise css errors might happen
    EXPECT_TRUE(pred.booleanEvaluation(3, worldFourWay, egoVehicle, obs1));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldFourWay, egoVehicle, obs1));
    EXPECT_FALSE(pred.booleanEvaluation(0, worldFourWay, obs1, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldFourWay, obs1, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldFourWay, obs1, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldFourWay, obs1, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldFourWay, obs1, egoVehicle));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldFourWay, egoVehicle, obs2));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldFourWay, egoVehicle, obs2));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldFourWay, egoVehicle, obs2));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldFourWay, egoVehicle, obs2));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldFourWay, egoVehicle, obs2));
    EXPECT_TRUE(pred.booleanEvaluation(0, worldFourWay, obs2, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldFourWay, obs2, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldFourWay, obs2, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldFourWay, obs2, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(4, worldFourWay, obs2, egoVehicle));

    EXPECT_TRUE(pred.booleanEvaluation(0, worldFourWay, egoVehicle, obs3));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldFourWay, egoVehicle, obs3));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldFourWay, egoVehicle, obs3));
    EXPECT_TRUE(pred.booleanEvaluation(3, worldFourWay, egoVehicle, obs3));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldFourWay, egoVehicle, obs3));
    EXPECT_TRUE(pred.booleanEvaluation(0, worldFourWay, obs3, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldFourWay, obs3, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldFourWay, obs3, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldFourWay, obs3, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(4, worldFourWay, obs3, egoVehicle));
}

TEST_F(TestOnSameRoadPredicate, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(TestOnSameRoadPredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}
