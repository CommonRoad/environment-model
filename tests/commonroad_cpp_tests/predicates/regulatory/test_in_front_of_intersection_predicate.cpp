#include "test_in_front_of_intersection_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void InFrontOfIntersectionPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_Intersection.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 17.0, 0.0, 0, 0, 0, 0, 17, 0.0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 16.0, 0, 0, 0, 0, 0, 16, 0.0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 33.0, 3, 0, 0, M_PI, 0, 33.0, 0.0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 3, 0, 0, 0, 0, 10, 0.0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 17.4, 0, 0, 0, 0, 0, 17.4, 0.0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 23, 0, 0, 0, 0, 0, 23, 0.0);

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

TEST_F(InFrontOfIntersectionPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne));  // half a meter from intersection away
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleTwo)); // four meter from intersection away

    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));  // 0m away from intersection
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleTwo)); // too far away from intersection

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));  // 0m away from intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo)); // already in intersection
}

TEST_F(InFrontOfIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(InFrontOfIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}