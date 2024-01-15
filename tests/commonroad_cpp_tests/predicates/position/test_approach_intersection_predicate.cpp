#include "test_approach_intersection_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void ApproachIntersectionPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() +
                               "/predicates/DEU_TrafficLightTest-1/DEU_TrafficLightTest-1_1_T-1.pb"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 23.5, 10.0, 0, 0, M_PI / 2, 0, 23.5, 10.0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 23.5, 3, 0, 0, 0, 0, 23.5, 3);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 5, 0, 0, 0, 0, 0, 5, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 26.5, 0, 0, 0, 0, 0, 26, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 21, 0, 0, 0, M_PI / 2, 0, 21.0, 2);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 26.5, -10.0, 0, 0, M_PI / 2, 0, 26, -10.0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    Obstacle::state_map_t trajectoryPredictionObstacleTwo{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {}, timeStepSize));
}

TEST_F(ApproachIntersectionPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(
        0, world, obstacleOne)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleTwo)); // completely on intersection
    EXPECT_TRUE(pred.booleanEvaluation(
        1, world, obstacleOne)); // in front of intersection/traffic light -> completely on incoming
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleTwo)); // completely on intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne)); // on intersection and on incoming
    EXPECT_TRUE(pred.booleanEvaluation(
        2, world, obstacleTwo)); // in front of intersection/traffic light -> completely on incoming
}

TEST_F(ApproachIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(ApproachIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}