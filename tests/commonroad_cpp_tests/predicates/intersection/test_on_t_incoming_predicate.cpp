#include "test_on_t_incoming_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void OnTIncomingPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/USA_t_intersection-1_1_T-2.xml"};
    std::string pathToTestFile_2{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_Intersection.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    const auto &[obstacles_2, roadNetwork_2, timeStepSize_2] = InputUtils::getDataFromCommonRoad(pathToTestFile_2);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 23.5, 15, 0, 0, -M_PI / 2, 0, 14, 0.0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 0, 0, 0, 0, 0, 10, 0.0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 26.5, -12.5, 0, 0, 0, M_PI, 26.5, -12.5);

    Obstacle::state_map_t trajectoryPredictionObstacleOne{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    obstacleTwo = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, timeStepSize));

    world_2 = std::make_shared<World>(
        World("testWorld", 0, roadNetwork_2, std::vector<std::shared_ptr<Obstacle>>{obstacleTwo}, {}, timeStepSize_2));
}

TEST_F(OnTIncomingPredicateTest, BooleanEvaluation) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world_2, obstacleTwo)); // approaches no t-intersection
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne));   // approaches t-intersection but not t-incoming
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne));    // approaches t-intersection and on t-incoming
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne));   // approaches t-intersection but not on t-incoming
}

TEST_F(OnTIncomingPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne), std::runtime_error);
}

TEST_F(OnTIncomingPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne), std::runtime_error);
}
