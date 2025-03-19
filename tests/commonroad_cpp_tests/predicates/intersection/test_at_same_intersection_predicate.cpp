#include "test_at_same_intersection_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void AtSameIntersectionPredicateTest::SetUp() {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_Intersection.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 26.5, -6.0, 0, 0, M_PI / 2, 0, 26.5, -6.0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 22.5, 0, 0, 0, 0, 0, 25, 0.0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 23.5, 10, 0, 0, -M_PI / 2, 0, 23.5, 10.0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 25.5, 3, 0, 0, -M_PI / 2, 0, 25.5, 3.0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 8, 0, 0, 0, 0, 0, 8, 0.0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 5, 0, 0, 0, 0, 0, 5, 0.0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));

    world = std::make_shared<World>(World(
        "testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {obstacleTwo}, timeStepSize));
}

TEST_F(AtSameIntersectionPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo));
}

TEST_F(AtSameIntersectionPredicateTest, BooleanEvaluationDifferentIntersection) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/DEU_Guetersloh-25_4_T-1.xml"};
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestFile);
    auto worldDEU{
        std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto obs1{worldDEU->findObstacle(32)};
    auto obs2{worldDEU->findObstacle(370)};
    auto obs3{worldDEU->findObstacle(330)};
    EXPECT_FALSE(pred.booleanEvaluation(0, worldDEU, obs1, obs2));
    EXPECT_FALSE(pred.booleanEvaluation(0, worldDEU, obs2, obs1));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldDEU, obs1, obs3));
    EXPECT_FALSE(pred.booleanEvaluation(0, worldDEU, obs3, obs1));

    EXPECT_TRUE(pred.booleanEvaluation(0, worldDEU, obs3, obs2));
    EXPECT_TRUE(pred.booleanEvaluation(0, worldDEU, obs2, obs3));
}

TEST_F(AtSameIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(AtSameIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}
