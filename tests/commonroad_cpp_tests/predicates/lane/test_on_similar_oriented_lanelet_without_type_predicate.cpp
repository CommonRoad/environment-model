#include "test_on_similar_oriented_lanelet_without_type_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void OnSimilarOrientedLaneletWithoutTypePredicateTest::SetUp() {}

TEST_F(OnSimilarOrientedLaneletWithoutTypePredicateTest, BooleanEvaluationOnShoulder) {}

TEST_F(OnSimilarOrientedLaneletWithoutTypePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(OnSimilarOrientedLaneletWithoutTypePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(OnSimilarOrientedLaneletWithoutTypePredicateTest, SetBasedPredictionIntersection) {
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

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, {}, {"left"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, ego, {}, {"left"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obs1, {}, {"left"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obs1, {}, {"straight"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, {}, {"right"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(40, world, ego, {}, {"right"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obs1, {}, {"left"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obs1, {}, {"right"}, true));
}
