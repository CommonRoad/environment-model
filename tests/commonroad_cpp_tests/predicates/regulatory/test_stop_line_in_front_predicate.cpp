#include "test_stop_line_in_front_predicate.h"
#include "../utils_predicate_test.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void StopLineInFrontPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 5, 2, 17.5, 0, 0, 0, 5, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 5, 6, 17.5, 0, 0, 0, 5, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 17.4, 2, 2.5, 0, 0, 0, 17.4, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 17.4, 6, 0.5, 0, 0, 0, 17.4, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 20, 2, 5.0, 0, 0, 0, 20, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 17.55, 6, 4.95, 0, 0, 0, 17.55, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 25, 2, 10.0, 0, 0, 0, 25, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 25, 6, 10.0, 0, 0, 0, 25, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_2()};

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, 0.1));
}

TEST_F(StopLineInFrontPredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // stop line completely in front
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));  // stop line exactly at obstacle front
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne)); // obstacle on stop line
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne)); // stop line behind obstacle
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleTwo)); // stop line completely in front
    EXPECT_TRUE(pred.booleanEvaluation(
        1, world, obstacleTwo)); // stop line exactly at obstacle front (on min lon. position of stop line)
    EXPECT_TRUE(pred.booleanEvaluation(
        2, world, obstacleTwo)); // stop line with two different lon. positions; obstacle above stop line
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleTwo)); // stop line behind obstacle
}

TEST_F(StopLineInFrontPredicateTest, TestScenario1) {
    std::array<std::string, 1> scenarios{"DEU_testStopLine-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() +
                                  "/predicates/DEU_TestStopLine-1/DEU_TestStopLine-1_1_T-1.pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world{std::make_shared<World>(
        World("testWorld", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne))};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstaclesScenarioOne.at(3)));

    EXPECT_TRUE(pred.booleanEvaluation(39, world, obstaclesScenarioOne.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(39, world, obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(39, world, obstaclesScenarioOne.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(39, world, obstaclesScenarioOne.at(3)));

    EXPECT_FALSE(pred.booleanEvaluation(40, world, obstaclesScenarioOne.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, obstaclesScenarioOne.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obstaclesScenarioOne.at(3)));

    EXPECT_FALSE(pred.booleanEvaluation(41, world, obstaclesScenarioOne.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(41, world, obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(41, world, obstaclesScenarioOne.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(41, world, obstaclesScenarioOne.at(3)));

    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstaclesScenarioOne.at(3)));

    EXPECT_FALSE(pred.booleanEvaluation(69, world, obstaclesScenarioOne.at(0)));
    EXPECT_TRUE(pred.booleanEvaluation(69, world, obstaclesScenarioOne.at(1)));
    EXPECT_TRUE(pred.booleanEvaluation(69, world, obstaclesScenarioOne.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(69, world, obstaclesScenarioOne.at(3)));

    EXPECT_FALSE(pred.booleanEvaluation(70, world, obstaclesScenarioOne.at(0)));
    EXPECT_TRUE(pred.booleanEvaluation(70, world, obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(70, world, obstaclesScenarioOne.at(2)));
    EXPECT_TRUE(pred.booleanEvaluation(70, world, obstaclesScenarioOne.at(3)));

    EXPECT_FALSE(pred.booleanEvaluation(150, world, obstaclesScenarioOne.at(0)));
    EXPECT_TRUE(pred.booleanEvaluation(150, world, obstaclesScenarioOne.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(150, world, obstaclesScenarioOne.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(150, world, obstaclesScenarioOne.at(3)));
}

TEST_F(StopLineInFrontPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(StopLineInFrontPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}