#include "test_causes_braking_intersection_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void CausesBrakingIntersectionPredicateTest::SetUp() {}

TEST_F(CausesBrakingIntersectionPredicateTest, BooleanEvaluation) {
    // TODO
}

TEST_F(CausesBrakingIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world1, obstacleOne), std::runtime_error);
}

TEST_F(CausesBrakingIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world1, obstacleOne), std::runtime_error);
}

TEST_F(CausesBrakingIntersectionPredicateTest, SetBasedPrediction) {
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

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(20, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(29, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(38, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obs1, ego, {}, true));

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs1, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(20, world, ego, obs1, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(29, world, ego, obs1, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(38, world, ego, obs1, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, ego, obs1, {}, true));
}
