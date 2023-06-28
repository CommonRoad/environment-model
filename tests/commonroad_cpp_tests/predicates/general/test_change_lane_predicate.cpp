#include "test_change_lane_predicate.h"

#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"

void ChangeLanePredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 90, 2, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 95, 2, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 100, 2, 5, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 105, 2, 5, 0, 0);

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 85, 2, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 90, 2, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 95, 4, 5, 0, M_PI_4);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 100, 6, 5, 0, 0);

    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 85, 6, 5, 0, 0);
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 90, 6, 5, 0, 0);
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 95, 4, 5, 0, M_PI + M_PI_2 + M_PI_4);
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 100, 2, 5, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleThree),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree)};

    Obstacle::state_map_t trajectoryPredictionObstacleFour;
    for (int i = 0; i <= 50; ++i) {
        std::shared_ptr<State> newState = std::make_shared<State>(0, i, 1.5, 10, 0, 0);
        trajectoryPredictionObstacleFour.insert(std::pair<int, std::shared_ptr<State>>(i, newState));
    }

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    obstacleFour = std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, trajectoryPredictionObstacleFour[0],
                                                       ObstacleType::car, 50, 10, 3, -10, 0.3,
                                                       trajectoryPredictionObstacleFour, 5, 2));

    // World1 with just two parallel lanelets
    auto roadNetwork1{utils_predicate_test::create_road_network()};
    world1 = std::make_shared<World>(World(0, roadNetwork1, {obstacleOne}, {obstacleTwo, obstacleThree}, 0.1));

    // World2 with 2 lanelets followed by 1 uccessor each
    auto roadNetwork2{utils_predicate_test::create_road_network_with_2_successors(
        {LaneletType::urban}, {LaneletType::urban}, {LaneletType::urban}, {LaneletType::urban})};
    world2 = std::make_shared<World>(World(0, roadNetwork2, {obstacleOne}, {obstacleTwo, obstacleThree}, 0.1));

    // Worlds with access- and exit-ramps on interstate
    pathToTestFileAccessRamp =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_testConsiderEnteringVehiclesForLaneChange-1";
    pathToTestFileExitRamp = TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TestOvertakingExitRamp-1";
    pathToTestFileTwoFollowingLaneletsAndAccessRampFalse =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_testOnFollowingLaneletsAndAccessRampFalse-1";
    pathToTestFileTwoFollowingLaneletsAndAccessRampTrue =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_testOnFollowingLaneletsAndAccessRampTrue-1";

    optLeft = std::make_shared<OptionalPredicateParameters>();
    optLeft->turningDirection = {TurningDirection::left};
    optRight = std::make_shared<OptionalPredicateParameters>();
    optRight->turningDirection = {TurningDirection::right};
}

TEST_F(ChangeLanePredicateTest, TwoParallelLaneletsNoChange) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(2, world1, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleOne, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleOne, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(2, world1, obstacleOne, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleOne, {}, optRight));
}

TEST_F(ChangeLanePredicateTest, TwoParallelLaneletsLeftChange) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleTwo, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleTwo, {}, optLeft));
    EXPECT_TRUE(pred.booleanEvaluation(2, world1, obstacleTwo, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleTwo, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleTwo, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleTwo, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(2, world1, obstacleTwo, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleTwo, {}, optRight));
}

TEST_F(ChangeLanePredicateTest, TwoParallelLaneletsRightChange) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(2, world1, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleThree, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleThree, {}, optRight));
    EXPECT_TRUE(pred.booleanEvaluation(2, world1, obstacleThree, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleThree, {}, optRight));
}

TEST_F(ChangeLanePredicateTest, TwoParallelLaneletsWithSuccessorsNoChange) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, obstacleOne, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, obstacleOne, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, obstacleOne, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, obstacleOne, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, obstacleOne, {}, optRight));
}

TEST_F(ChangeLanePredicateTest, TwoParallelLaneletsWithSuccessorsLeftChange) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, obstacleTwo, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, obstacleTwo, {}, optLeft));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, obstacleTwo, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, obstacleTwo, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, obstacleTwo, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, obstacleTwo, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, obstacleTwo, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, obstacleTwo, {}, optRight));
}

TEST_F(ChangeLanePredicateTest, TwoParallelLaneletsWithSuccessorsRightChange) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, obstacleThree, {}, optLeft));
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, obstacleThree, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, obstacleThree, {}, optRight));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, obstacleThree, {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, obstacleThree, {}, optRight));
}

TEST_F(ChangeLanePredicateTest, AccessRampNoChange) {
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFileAccessRamp);
    std::shared_ptr<World> world = std::make_shared<World>(World(0, roadNetwork, {obstacles[0]}, {}, 0.1));
    EXPECT_EQ(obstacles[0]->getId(), 1000);
    EXPECT_EQ(obstacles[0]->getStateByTimeStep(57)->getXPosition(), 167.5);
    for (int i = 0; i < 61; ++i) {
        EXPECT_FALSE(pred.booleanEvaluation(i, world, obstacles[0], {}, optLeft));
    }
    for (int i = 0; i < 15; ++i) {
        EXPECT_FALSE(pred.booleanEvaluation(i, world, obstacles[0], {}, optRight));
    }
    EXPECT_TRUE(pred.booleanEvaluation(15, world, obstacles[0], {}, optRight));
    EXPECT_TRUE(pred.booleanEvaluation(16, world, obstacles[0], {}, optRight));
    for (int i = 17; i < 61; ++i) {
        EXPECT_FALSE(pred.booleanEvaluation(i, world, obstacles[0], {}, optRight));
    }
}

TEST_F(ChangeLanePredicateTest, ExitRampNoChange) {
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFileExitRamp);
    std::shared_ptr<World> world = std::make_shared<World>(World(0, roadNetwork, {obstacleFour}, {}, 0.1));
    for (int i = 0; i < 51; ++i) {
        EXPECT_FALSE(pred.booleanEvaluation(i, world, obstacleFour, {}, optLeft));
        EXPECT_FALSE(pred.booleanEvaluation(i, world, obstacleFour, {}, optRight));
    }
}

TEST_F(ChangeLanePredicateTest, OnFollowingLaneletsAndAccessRampConnected) {
    const auto &[obstacles, roadNetwork, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFileTwoFollowingLaneletsAndAccessRampFalse);
    std::shared_ptr<World> world = std::make_shared<World>(World(0, roadNetwork, {obstacles[0]}, {}, 0.1));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles[0], {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles[0], {}, optLeft));
}

TEST_F(ChangeLanePredicateTest, OnFollowingLaneletsAndAccessRampNotConnected) {
    const auto &[obstacles, roadNetwork, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFileTwoFollowingLaneletsAndAccessRampTrue);
    std::shared_ptr<World> world = std::make_shared<World>(World(0, roadNetwork, {obstacles[0]}, {}, 0.1));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles[0], {}, optRight));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles[0], {}, optLeft));
}