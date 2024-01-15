#include "test_close_to_crosswalk_predicate.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"

void ClossToCrosswalkPredicateTest::SetUp() {
    std::string pathToTestFile{
        TestUtils::getTestScenarioDirectory() +
        "/predicates/DEU_TestNoOvertakingCrosswalk-1_1_T-1.xml"}; // not an uncontrolled intersection
    const auto &[obstacles, roadNetwork, timeStepSize] = InputUtils::getDataFromCommonRoad(pathToTestFile);

    obstacleOne = obstacles[0];
    obstacleTwo = obstacles[1];
    world = std::make_shared<World>(
        World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, timeStepSize));
}

TEST_F(ClossToCrosswalkPredicateTest, BooleanEvaluation) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne));   // not at uncontrolled intersection
    EXPECT_TRUE(pred.booleanEvaluation(5, world, obstacleOne));   // approach uncontrolled intersection
    EXPECT_TRUE(pred.booleanEvaluation(10, world, obstacleOne));  // in uncontrolled intersection but does not approach
    EXPECT_TRUE(pred.booleanEvaluation(15, world, obstacleOne));  // in uncontrolled intersection but does not approach
    EXPECT_TRUE(pred.booleanEvaluation(20, world, obstacleOne));  // approach uncontrolled intersection
    EXPECT_FALSE(pred.booleanEvaluation(35, world, obstacleOne)); // approach uncontrolled intersection
    pred.getParameters().updateParam("dCloseToCrossing", 5.0);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne));  // not at uncontrolled intersection
    EXPECT_FALSE(pred.booleanEvaluation(5, world, obstacleOne));  // approach uncontrolled intersection
    EXPECT_FALSE(pred.booleanEvaluation(10, world, obstacleOne)); // in uncontrolled intersection but does not approach
    EXPECT_TRUE(pred.booleanEvaluation(15, world, obstacleOne));  // in uncontrolled intersection but does not approach
    EXPECT_TRUE(pred.booleanEvaluation(20, world, obstacleOne));  // approach uncontrolled intersection
    EXPECT_FALSE(pred.booleanEvaluation(35, world, obstacleOne)); // approach uncontrolled intersection
}

TEST_F(ClossToCrosswalkPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(ClossToCrosswalkPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}