#include "test_has_priority_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

TEST_F(HasPriorityPredicateTest, BooleanEvaluation) {
    // Todo
}

TEST_F(HasPriorityPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(HasPriorityPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(HasPriorityPredicateTest, SetBasedPrediction) {
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

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs1, {"left", "straight"}, true));
}
