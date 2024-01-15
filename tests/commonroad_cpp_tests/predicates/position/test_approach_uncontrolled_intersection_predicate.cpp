#include "test_approach_uncontrolled_intersection_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void ApproachUncontrolledIntersectionPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() +
                               "/predicates/DEU_Intersection.xml"}; // not an uncontrolled intersection
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::string pathToTestFile_2{TestUtils::getTestScenarioDirectory() + "/predicates/uncontrolled_intersection.xml"};
    const auto &[obstacles_2, roadNetwork_2, timeStepSize_2] = InputUtils::getDataFromCommonRoad(pathToTestFile_2);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 17.0, 0.0, 0, 0, 0, 0, 17, 0.0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 15, 0, 0, 0, 0, 0, 15, 0.0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 26.5, 0, 0, 0, M_PI / 2, 0, 26, 0.0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 23.5, 0, 0, 0, -M_PI / 2, 0, 26, 0.0);
    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(3, 40.5, 3, 0, 0, M_PI, 0, 9.5, 0.0);
    std::shared_ptr<State> stateFiveObstacleOne = std::make_shared<State>(3, 23.5, 15, 0, 0, -M_PI / 2, 0, 10.5, 0.0);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, timeStepSize));

    world_2 = std::make_shared<World>(
        World(0, roadNetwork_2, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, timeStepSize));
}

TEST_F(ApproachUncontrolledIntersectionPredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne)); // not at uncontrolled intersection

    EXPECT_TRUE(pred.booleanEvaluation(1, world_2, obstacleOne)); // approach uncontrolled intersection

    EXPECT_FALSE(pred.booleanEvaluation(2, world_2, obstacleOne)); // in uncontrolled intersection but does not approach
    EXPECT_FALSE(pred.booleanEvaluation(3, world_2, obstacleOne)); // in uncontrolled intersection but does not approach

    EXPECT_TRUE(pred.booleanEvaluation(4, world_2, obstacleOne)); // approach uncontrolled intersection
    EXPECT_TRUE(pred.booleanEvaluation(5, world_2, obstacleOne)); // approach uncontrolled intersection
}

TEST_F(ApproachUncontrolledIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(ApproachUncontrolledIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}