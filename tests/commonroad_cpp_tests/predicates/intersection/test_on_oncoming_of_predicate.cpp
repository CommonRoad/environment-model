#include "test_on_oncoming_of_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "commonroad_cpp/obstacle/obstacle.h"

void OnOncomingOfPredicateTest::SetUp() {}

TEST_F(OnOncomingOfPredicateTest, BooleanEvaluationOncoming) {
    auto pathToTestFile =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TestTurnLeft-1/DEU_TestTurnLeft-1_2_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork,
                                          std::vector<std::shared_ptr<Obstacle>>{obstacles.at(0), obstacles.at(1)}, {},
                                          timeStepSize));

    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(0), obstacles.at(1), {"0.75"}));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
    EXPECT_TRUE(pred.booleanEvaluation(20, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
    EXPECT_TRUE(pred.booleanEvaluation(20, world, obstacles.at(0), obstacles.at(1), {"0.75"}));

    EXPECT_TRUE(pred.booleanEvaluation(38, world, obstacles.at(0), obstacles.at(1), {"0.75"}));
    EXPECT_TRUE(pred.booleanEvaluation(38, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
    EXPECT_TRUE(pred.booleanEvaluation(50, world, obstacles.at(0), obstacles.at(1), {"0.75"}));
    EXPECT_TRUE(pred.booleanEvaluation(50, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
}

TEST_F(OnOncomingOfPredicateTest, BooleanEvaluationNotOncoming) {
    auto pathToTestFile = TestUtils::getTestScenarioDirectory() +
                          "/predicates/DEU_TestRightBeforeLeft-1/DEU_TestRightBeforeLeft-1_2_T-1.pb";
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork,
                                          std::vector<std::shared_ptr<Obstacle>>{obstacles.at(0), obstacles.at(1)}, {},
                                          timeStepSize));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles.at(0), obstacles.at(1), {"0.75"}));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
    EXPECT_FALSE(pred.booleanEvaluation(20, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
    EXPECT_FALSE(pred.booleanEvaluation(20, world, obstacles.at(0), obstacles.at(1), {"0.75"}));

    EXPECT_FALSE(pred.booleanEvaluation(38, world, obstacles.at(0), obstacles.at(1), {"0.75"}));
    EXPECT_FALSE(pred.booleanEvaluation(38, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
    EXPECT_FALSE(pred.booleanEvaluation(50, world, obstacles.at(0), obstacles.at(1), {"0.75"}));
    EXPECT_FALSE(pred.booleanEvaluation(50, world, obstacles.at(1), obstacles.at(0), {"0.75"}));
}

TEST_F(OnOncomingOfPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, {}, {}), std::runtime_error);
}

TEST_F(OnOncomingOfPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, {}, {}), std::runtime_error);
}

TEST_F(OnOncomingOfPredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_TestTurnLeft-2_1_S-2";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto obstacles{scenarioXml.obstacles};
    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, obstacles, {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(1001)};

    //    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, {obs1}, {"0.75"}, true));
    //    EXPECT_TRUE(pred.booleanEvaluation(40, world, ego, {obs1}, {"0.75"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs1, {ego}, {"0.75"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(20, world, obs1, {ego}, {"0.75"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(29, world, obs1, {ego}, {"0.75"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obs1, {ego}, {"0.75"}, true));
}
