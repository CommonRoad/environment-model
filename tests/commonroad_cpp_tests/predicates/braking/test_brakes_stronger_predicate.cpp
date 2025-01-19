#include "test_brakes_stronger_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include "commonroad_cpp/interfaces/commonroad/input_utils.h"

void BrakesStrongerPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 10, -2, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 10, 2, 10, -1, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 0, 2, 10, -1, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 2, 10, -2, 0);

    std::shared_ptr<State> stateTwoObstacleOne{std::make_shared<State>()};
    stateTwoObstacleOne->setTimeStep(2);
    stateTwoObstacleOne->setXPosition(20);
    stateTwoObstacleOne->setYPosition(2);
    stateTwoObstacleOne->setVelocity(10.1);
    stateTwoObstacleOne->setGlobalOrientation(0);
    std::shared_ptr<State> stateTwoObstacleTwo{std::make_shared<State>()};
    stateTwoObstacleTwo->setTimeStep(2);
    stateTwoObstacleTwo->setXPosition(20);
    stateTwoObstacleTwo->setYPosition(2);
    stateTwoObstacleTwo->setVelocity(10.1);
    stateTwoObstacleTwo->setGlobalOrientation(0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 30, 2, 10, 2, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 40, 2, 10, 1, 0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};

    world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                      std::vector<std::shared_ptr<Obstacle>>{obstacleTwo}, 0.1));
}

TEST_F(BrakesStrongerPredicateTest, BooleanEvaluation) {
    std::vector<std::string> opt{"0.0"};
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, obstacleTwo, opt));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo, opt));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, obstacleOne, obstacleTwo, opt));
}

TEST_F(BrakesStrongerPredicateTest, ConstraintEvaluation) {
    std::vector<std::string> opt{"0.0"};
    EXPECT_EQ(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, -1);
    EXPECT_EQ(pred.constraintEvaluation(1, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, -2);
    EXPECT_EQ(pred.constraintEvaluation(2, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, 0);
    EXPECT_EQ(pred.constraintEvaluation(3, world, obstacleOne, obstacleTwo, opt).realValuedConstraint, 0);
}

TEST_F(BrakesStrongerPredicateTest, RobustEvaluation) {
    std::vector<std::string> opt{"0.0"};
    EXPECT_EQ(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo, opt), 1);
    EXPECT_EQ(pred.robustEvaluation(1, world, obstacleOne, obstacleTwo, opt), -1);
    EXPECT_NEAR(pred.robustEvaluation(2, world, obstacleOne, obstacleTwo, opt), -1, 0.0001);
    EXPECT_EQ(pred.robustEvaluation(3, world, obstacleOne, obstacleTwo, opt), -2);
}

TEST_F(BrakesStrongerPredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto worldTmp{
        std::make_shared<World>(World("testWorld", 0, std::get<1>(scenarioXml), std::get<0>(scenarioXml), {}, 0.1))};
    auto obs1{worldTmp->findObstacle(100)};
    auto ego{worldTmp->findObstacle(42)};

    EXPECT_FALSE(pred.booleanEvaluation(0, worldTmp, obs1, ego, {"0.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldTmp, obs1, ego, {"0.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, worldTmp, obs1, ego, {"0.0"}, true));

    EXPECT_FALSE(pred.booleanEvaluation(0, worldTmp, ego, obs1, {"0.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldTmp, ego, obs1, {"0.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(30, worldTmp, ego, obs1, {"0.0"}, true));
}
