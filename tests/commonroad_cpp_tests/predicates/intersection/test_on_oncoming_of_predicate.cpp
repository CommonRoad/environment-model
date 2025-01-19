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
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
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
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);
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
