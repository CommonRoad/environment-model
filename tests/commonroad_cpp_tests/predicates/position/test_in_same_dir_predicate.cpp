#include "test_in_same_dir_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void InSameDirPredicateTest::SetUp() {

    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_Intersection.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 5, 0, 10, 0, 0, 0, 5, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 15, 0, 10, -1, 0, 0, 15, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 17, 0.0, 0, 0, 0, 0, 17, 0.0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 17, 3, 0, 0, M_PI, 0, 28.5, 0.0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 40, 3.0, 0, 0, M_PI, 0, 5.5, 0.0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 23.5, 10, 0, 0, -M_PI, 0, 16.5, 0.0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 10, 0.0, 0, 0, 0, 0.1, 10, 0.0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 15, 3, 0, 0, 0, 0, 15, 0.0);

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

    world = std::make_shared<World>(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                    std::vector<std::shared_ptr<Obstacle>>{obstacleTwo}, 0.1);
}

TEST_F(InSameDirPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, obstacleTwo));
}

TEST_F(InSameDirPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(InSameDirPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}