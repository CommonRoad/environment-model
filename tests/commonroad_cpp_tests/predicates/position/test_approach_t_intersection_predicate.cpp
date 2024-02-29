#include "test_approach_t_intersection_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void ApproachTIntersectionPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/T_intersection.xml"};
    std::string pathToTestFile_2{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_Intersection.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    const auto &[obstacles_2, roadNetwork_2, timeStepSize_2] = InputUtils::getDataFromCommonRoad(pathToTestFile_2);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 17, 0.0, 0, 0, 0, 0, 17, 0.0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 25, 0, 0, 0, 0, 0, 25, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 23, 3, 0, 0, M_PI, 0, 23, 0.0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 40, 3, 0, 0, M_PI, 0, 10, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 23.5, 15, 0, 0, -M_PI / 2, 0, 3, 0.0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 23.5, 30, 0, 0, -M_PI / 2, 0, -3.5, 0);

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

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork,
                                          std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {},
                                          timeStepSize));

    world_2 = std::make_shared<World>(World("testWorld", 0, roadNetwork_2,
                                            std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo}, {},
                                            timeStepSize_2));
}

TEST_F(ApproachTIntersectionPredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne));

    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne));

    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleTwo));

    EXPECT_FALSE(pred.booleanEvaluation(2, world_2, obstacleOne)); // not on t-intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, world_2, obstacleTwo)); // not on t-intersection
}

TEST_F(ApproachTIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(ApproachTIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}