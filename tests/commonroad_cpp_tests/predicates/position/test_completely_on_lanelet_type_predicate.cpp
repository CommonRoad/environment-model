#include "test_completely_on_lanelet_type_predicate.h"
#include "../utils_predicate_test.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void CompletelyOnLaneletTypePredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 50, 6, 25, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 75, 5, 25, 0, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 100, 2, 25, 0, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 125, 2, 25, 0, 0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 150, 4, 25, 0, 0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(4, 175, 6, 25, 0, 0);

    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 155, -1.5, 25, 0, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 161, -1.4, 25, 0, 0.3);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 167, -0.5, 25, 0, 0.3);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 173, 0.5, 25, 0, 0.3);
    std::shared_ptr<State> stateFourObstacleTwo = std::make_shared<State>(4, 179, 1.4, 25, 0, 0);
    std::shared_ptr<State> stateFiveObstacleTwo = std::make_shared<State>(4, 185, 1.5, 25, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    // world 1
    auto roadNetwork{utils_predicate_test::create_road_network_with_2_successors(
        {LaneletType::urban}, {LaneletType::urban}, {LaneletType::intersection}, {LaneletType::mainCarriageWay})};
    world1 = std::make_shared<World>(World(0, roadNetwork, {obstacleOne}, {}, 0.1));

    // world 2
    auto pathToTestFileAccessRamp =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_testOnFollowingLaneletsAndAccessRampFalse-1";
    const auto &[obstacles, roadNetwork2, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFileAccessRamp);
    world2 = std::make_shared<World>(World(0, roadNetwork2, {}, {}, 0.1));
}

TEST_F(CompletelyOnLaneletTypePredicateTest, CheckOnTwoParallelLanes) {
    auto optInt = std::make_shared<OptionalPredicateParameters>();
    optInt->laneletType = {LaneletType::intersection};
    auto optMain = std::make_shared<OptionalPredicateParameters>();
    optMain->laneletType = {LaneletType::mainCarriageWay};
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleOne, {}, optInt));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleOne, {}, optInt));
    EXPECT_FALSE(pred.booleanEvaluation(2, world1, obstacleOne, {}, optInt));
    EXPECT_TRUE(pred.booleanEvaluation(3, world1, obstacleOne, {}, optInt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world1, obstacleOne, {}, optInt));
    EXPECT_FALSE(pred.booleanEvaluation(5, world1, obstacleOne, {}, optInt));
    EXPECT_FALSE(pred.booleanEvaluation(0, world1, obstacleOne, {}, optMain));
    EXPECT_FALSE(pred.booleanEvaluation(1, world1, obstacleOne, {}, optMain));
    EXPECT_FALSE(pred.booleanEvaluation(2, world1, obstacleOne, {}, optMain));
    EXPECT_FALSE(pred.booleanEvaluation(3, world1, obstacleOne, {}, optMain));
    EXPECT_FALSE(pred.booleanEvaluation(4, world1, obstacleOne, {}, optMain));
    EXPECT_TRUE(pred.booleanEvaluation(5, world1, obstacleOne, {}, optMain));
}

TEST_F(CompletelyOnLaneletTypePredicateTest, CheckOnMergingLanelets) {
    auto optAcc = std::make_shared<OptionalPredicateParameters>();
    optAcc->laneletType = {LaneletType::accessRamp};
    auto optMain = std::make_shared<OptionalPredicateParameters>();
    optMain->laneletType = {LaneletType::mainCarriageWay};
    EXPECT_TRUE(pred.booleanEvaluation(0, world2, obstacleTwo, {}, optAcc));
    EXPECT_TRUE(pred.booleanEvaluation(1, world2, obstacleTwo, {}, optAcc));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, obstacleTwo, {}, optAcc));
    EXPECT_TRUE(pred.booleanEvaluation(3, world2, obstacleTwo, {}, optAcc));
    EXPECT_FALSE(pred.booleanEvaluation(4, world2, obstacleTwo, {}, optAcc));
    EXPECT_FALSE(pred.booleanEvaluation(5, world2, obstacleTwo, {}, optAcc));
    EXPECT_FALSE(pred.booleanEvaluation(0, world2, obstacleTwo, {}, optMain));
    EXPECT_FALSE(pred.booleanEvaluation(1, world2, obstacleTwo, {}, optMain));
    EXPECT_FALSE(pred.booleanEvaluation(2, world2, obstacleTwo, {}, optMain));
    EXPECT_FALSE(pred.booleanEvaluation(3, world2, obstacleTwo, {}, optMain));
    EXPECT_TRUE(pred.booleanEvaluation(4, world2, obstacleTwo, {}, optMain));
    EXPECT_TRUE(pred.booleanEvaluation(5, world2, obstacleTwo, {}, optMain));
}
