#include "test_in_front_of_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

#include "commonroad_cpp/obstacle/occupancy.h"

void TestInFrontOfPredicate::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 8, 0, 10, -1, 0, 0, 8, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 0, 0, 10, -6, 0, 0, 0, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 0, 4, 6, 0, 0, 10, 0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 12, 0, 10, 0, 0, 0, 12, 0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 14, 0, 10, -5, 0, 0, 14, 0);

    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 14, 0, 10, 0, 0, 0, 14, 0);
    std::shared_ptr<State> stateThreeObstacleTwo = std::make_shared<State>(3, 24, 0, 5, 0, 0, 0, 24, 0);

    std::shared_ptr<State> stateFourObstacleOne = std::make_shared<State>(4, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(4, 29, 0, 5, -8, 0, 0, 29, 0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleOne)};

    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleTwo)};

    state_map_t trajectoryPredictionObstacleThree{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleThree)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2));
    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    auto roadNetwork{utils_predicate_test::create_road_network()};

    world =
        std::make_shared<World>(World("testWorld", 0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne},
                                      std::vector<std::shared_ptr<Obstacle>>{obstacleTwo, obstacleThree}, 0.1));
}

TEST_F(TestInFrontOfPredicate, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, obstacleOne, obstacleTwo)); // ego vehicle behind
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne,
                                        obstacleTwo)); // ego vehicle and other vehicle have same occupancy
    EXPECT_FALSE(pred.booleanEvaluation(2, world, obstacleOne, obstacleTwo)); // ego vehicle is not completely in front
    EXPECT_TRUE(pred.booleanEvaluation(3, world, obstacleOne, obstacleTwo));  // ego vehicle is in front in same lane
    EXPECT_TRUE(
        pred.booleanEvaluation(4, world, obstacleOne, obstacleThree)); // ego vehicle is in front in another lane
}

TEST_F(TestInFrontOfPredicate, BooleanEvaluationValues) {
    EXPECT_FALSE(pred.booleanEvaluation(8, 0, 5, 5));
    EXPECT_FALSE(pred.booleanEvaluation(10, 10, 5, 5));
    EXPECT_FALSE(pred.booleanEvaluation(12, 14, 5, 5));
    EXPECT_TRUE(pred.booleanEvaluation(14, 24, 5, 5));
    EXPECT_TRUE(pred.booleanEvaluation(10, 29, 5, 5));
}

TEST_F(TestInFrontOfPredicate, ConstraintEvaluationObjects) {
    EXPECT_NEAR(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo).realValuedConstraint, 13, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(1, world, obstacleOne, obstacleTwo).realValuedConstraint, 15, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(2, world, obstacleOne, obstacleTwo).realValuedConstraint, 17, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(3, world, obstacleOne, obstacleTwo).realValuedConstraint, 19, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(4, world, obstacleOne, obstacleThree).realValuedConstraint, 15, 0.001);
}

TEST_F(TestInFrontOfPredicate, ConstraintEvaluationValues) {
    EXPECT_NEAR(pred.constraintEvaluation(8, 5, 5).realValuedConstraint, 13, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(10, 5, 5).realValuedConstraint, 15, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(12, 5, 5).realValuedConstraint, 17, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(14, 5, 5).realValuedConstraint, 19, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(10, 5, 5).realValuedConstraint, 15, 0.001);
}

TEST_F(TestInFrontOfPredicate, RobustEvaluationObjects) {
    EXPECT_NEAR(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), -13.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(1, world, obstacleOne, obstacleTwo), -5.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(2, world, obstacleOne, obstacleTwo), -3.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(3, world, obstacleOne, obstacleTwo), 5.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(4, world, obstacleOne, obstacleThree), 14.0, 0.001);
}

TEST_F(TestInFrontOfPredicate, RobustEvaluationValues) {
    EXPECT_NEAR(pred.robustEvaluation(8, 0, 5, 5), -13.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(10, 10, 5, 5), -5.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(12, 14, 5, 5), -3.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(14, 24, 5, 5), 5.0, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(10, 29, 5, 5), 14.0, 0.001);
}

TEST_F(TestInFrontOfPredicate, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_FALSE(pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_EQ(stat->numExecutions, 1);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(1, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_EQ(stat->numExecutions, 2);
}

TEST_F(TestInFrontOfPredicate, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    std::shared_ptr<State> currentState = std::make_shared<State>(0, 100, 0, 10, 0, 0);
    state_map_t trajectoryPrediction{std::pair<size_t, std::shared_ptr<State>>(0, currentState)};
    std::shared_ptr<Obstacle> dynamicObstacle = std::make_shared<Obstacle>(Obstacle(
        1, ObstacleRole::DYNAMIC, currentState, ObstacleType::car, 50, 10, 3, -10, 0.3, trajectoryPrediction, 5, 2));

    auto obstacles{scenarioXml.obstacles};
    obstacles.push_back(dynamicObstacle);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, obstacles, {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(100)};
    auto obs2{world->findObstacle(101)};

    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, obs1, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, obs2, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(0, world, dynamicObstacle, obs1, {}, true));
}
