#include "test_on_incoming_of_intersection_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"

void OnIncomingOfIntersectionPredicateTest::SetUp() {}

TEST_F(OnIncomingOfIntersectionPredicateTest, BooleanEvaluationObjectsOneIncoming) {
    auto pathToTestFile =
        TestUtils::getTestScenarioDirectory() + "/DEU_TestInRightLane-1/DEU_TestInRightLane-1_1_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork,
                                          std::vector<std::shared_ptr<Obstacle>>{obstacles.at(0), obstacles.at(1)}, {},
                                          timeStepSize));

    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(0), {}, {"1002"}));
    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(1), {}, {"1002"}));
    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(1), {}, {"1002"}));
    EXPECT_FALSE(pred.booleanEvaluation(31, world, obstacles.at(0), {}, {"1002"}));

    EXPECT_TRUE(pred.booleanEvaluation(44, world, obstacles.at(0), {}, {"1002"}));
    EXPECT_TRUE(pred.booleanEvaluation(44, world, obstacles.at(1), {}, {"1002"}));
    EXPECT_TRUE(pred.booleanEvaluation(44, world, obstacles.at(0), {}, {"1002"}));
    EXPECT_TRUE(pred.booleanEvaluation(44, world, obstacles.at(1), {}, {"1002"}));
}

TEST_F(OnIncomingOfIntersectionPredicateTest, BooleanEvaluationObjectsTIntersection) {
    auto pathToTestFile = TestUtils::getTestScenarioDirectory() + "/USA_TIntersection-1/USA_TIntersection-1_1_T-1.pb";
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    const auto &[obstaclesTmp, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);

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

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(0), {}, {"1017"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(0), {}, {"1017"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(1), {}, {"1017"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(1), {}, {"1017"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(2), {}, {"1017"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(2), {}, {"1017"}));

    EXPECT_FALSE(pred.booleanEvaluation(15, world, obstacles.at(0), {}, {"1017"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacles.at(0), {}, {"1017"}));
    EXPECT_FALSE(pred.booleanEvaluation(15, world, obstacles.at(1), {}, {"1017"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obstacles.at(1), {}, {"1017"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacles.at(2), {}, {"1017"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacles.at(2), {}, {"1017"}));
}

TEST_F(OnIncomingOfIntersectionPredicateTest, BooleanEvaluationSeveralIntersections) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/ARG_Carcarana-6_5_T-1.xml"};
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestFile);
    auto worldDEU{
        std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};

    auto obs1{worldDEU->findObstacle(32)};
    auto obs2{worldDEU->findObstacle(315)};
    auto obs3{worldDEU->findObstacle(353)};
    EXPECT_TRUE(pred.booleanEvaluation(0, worldDEU, obs1, {}, {"8460"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, worldDEU, obs2, {}, {"8460"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, worldDEU, obs3, {}, {"8460"}));
}

TEST_F(OnIncomingOfIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, {}, {}), std::runtime_error);
}

TEST_F(OnIncomingOfIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, {}, {}), std::runtime_error);
}
