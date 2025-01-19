#include "test_on_incoming_left_of_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void OnIncomingLeftOfPredicateTest::SetUp() {}

TEST_F(OnIncomingLeftOfPredicateTest, BooleanEvaluationObjectsOneIncoming) {
    auto pathToTestFile =
        TestUtils::getTestScenarioDirectory() + "/DEU_TestInRightLane-1/DEU_TestInRightLane-1_1_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork,
                                          std::vector<std::shared_ptr<Obstacle>>{obstacles.at(0), obstacles.at(1)}, {},
                                          timeStepSize));

    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(0), obstacles.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(1), obstacles.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(1), obstacles.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(0), obstacles.at(1)));

    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstacles.at(0), obstacles.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstacles.at(1), obstacles.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstacles.at(0), obstacles.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(44, world, obstacles.at(1), obstacles.at(0)));
}

TEST_F(OnIncomingLeftOfPredicateTest, BooleanEvaluationObjectsTIntersection) {
    auto pathToTestFile = TestUtils::getTestScenarioDirectory() + "/USA_TIntersection-1/USA_TIntersection-1_1_T-1.pb";
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    const auto &[obstaclesTmp, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 26.5, -14.0, 0, 0, M_PI / 2);
    std::shared_ptr<State> state15ObstacleOne = std::make_shared<State>(1, 26.5, 15.0, 0, 0, M_PI / 2);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(1, state15ObstacleOne)};

    obstacles.push_back(obstaclesTmp.at(0));
    obstacles.push_back(obstaclesTmp.at(1));
    obstacles.push_back(
        std::make_shared<Obstacle>(Obstacle(124567, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car, 50,
                                            10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2)));

    world = std::make_shared<World>(World(
        "testWorld", 0, roadNetwork,
        std::vector<std::shared_ptr<Obstacle>>{obstacles.at(0), obstacles.at(1), obstacles.at(2)}, {}, timeStepSize));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(0), obstacles.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles.at(0), obstacles.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles.at(1), obstacles.at(0)));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(1), obstacles.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles.at(2), obstacles.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles.at(2), obstacles.at(2)));

    EXPECT_FALSE(pred.booleanEvaluation(15, world, obstacles.at(0), obstacles.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacles.at(0), obstacles.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(15, world, obstacles.at(1), obstacles.at(0)));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacles.at(1), obstacles.at(2)));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacles.at(2), obstacles.at(1)));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacles.at(2), obstacles.at(2)));
}

TEST_F(OnIncomingLeftOfPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, {}, {}), std::runtime_error);
}

TEST_F(OnIncomingLeftOfPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, {}, {}), std::runtime_error);
}
