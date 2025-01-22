#include "test_keeps_safe_distance_prec_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void KeepsSafeDistancePrecPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 0, 20, 0, 0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 20, 0, 20, -1, 0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 10, 0, 20, -5, 0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 20, 0, 20, 0, 0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 30, 0, 0, 0, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne)};
    state_map_t trajectoryPredictionOtherVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                 std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    obstacleTwo = std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionOtherVehicle, 5, 2));
    obstacleThree = std::make_shared<Obstacle>(
        Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10, 3, -10, 0.3, {}, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(
        World("testWorld", 0, roadNetwork,
              std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo, obstacleThree}, {}, 0.1));
}

TEST_F(KeepsSafeDistancePrecPredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleOne, obstacleTwo));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, obstacleOne, obstacleTwo));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, obstacleTwo, obstacleThree));
}

TEST_F(KeepsSafeDistancePrecPredicateTest, BooleanEvaluationValues) {
    EXPECT_TRUE(pred.booleanEvaluation(0, 20, 20, 20, -10, -10, 0.3, 5, 5));
    EXPECT_FALSE(pred.booleanEvaluation(20, 30, 20, 0, -10, -10, 0.3, 5, 5));
    EXPECT_TRUE(pred.booleanEvaluation(20, 10, 20, 10, -10, -10, 0.3, 5, 5));
}

TEST_F(KeepsSafeDistancePrecPredicateTest, ConstraintEvaluationObjects) {
    EXPECT_NEAR(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo).realValuedConstraint, 9.0, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(1, world, obstacleOne, obstacleTwo).realValuedConstraint, -1.0, 0.001);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, ConstraintEvaluationValues) {
    EXPECT_NEAR(pred.constraintEvaluation(20, 20, 20, -10, -10, 0.3, 5, 5).realValuedConstraint, 9.0, 0.001);
    EXPECT_NEAR(pred.constraintEvaluation(30, 20, 0, -10, -10, 0.3, 5, 5).realValuedConstraint, -1.0, 0.001);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, RobustEvaluationObjects) {
    EXPECT_NEAR(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), 9, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(1, world, obstacleOne, obstacleTwo), -21, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(0, world, obstacleTwo, obstacleThree), 15, 0.001);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, RobustEvaluationValues) {
    EXPECT_NEAR(pred.robustEvaluation(0, 20, 20, 20, -10, -10, 0.3, 5, 5), 9, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(20, 30, 20, 0, -10, -10, 0.3, 5, 5), -21, 0.001);
    EXPECT_NEAR(pred.robustEvaluation(20, 10, 20, 20, -10, -10, 0.3, 5, 5), 15, 0.001);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, StatisticBooleanEvaluation) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_EQ(stat->numExecutions, 1);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(1, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_EQ(stat->numExecutions, 2);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, obstacleTwo, timer, stat, obstacleThree));
    EXPECT_EQ(stat->numExecutions, 3);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, ComputeSafeDistance) {
    // both vehicles standing, no reaction time
    EXPECT_NEAR(KeepsSafeDistancePrecPredicate::computeSafeDistance(0, 0, -10, -10, 0.0), 0, 0.001);
    // both vehicles same velocity, no reaction time
    EXPECT_NEAR(KeepsSafeDistancePrecPredicate::computeSafeDistance(5, 5, -10, -10, 0.0), 0, 0.001);
    // both vehicles same velocity, with reaction time
    EXPECT_NEAR(KeepsSafeDistancePrecPredicate::computeSafeDistance(5, 5, -10, -10, 10.0), 50.0, 0.001);
    // following vehicle higher velocity, no reaction time
    EXPECT_NEAR(KeepsSafeDistancePrecPredicate::computeSafeDistance(10, 0, -10, -10, 0.0), 5.0, 0.001);
    // leading vehicle higher velocity, no reaction time
    EXPECT_NEAR(KeepsSafeDistancePrecPredicate::computeSafeDistance(0, 10, -10, -10, 0.0), -5.0, 0.001);
    // invalid acceleration
    EXPECT_THROW(KeepsSafeDistancePrecPredicate::computeSafeDistance(0, 0, 1, 2, 0.0), std::logic_error);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, ResetStatistics) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_EQ(stat->numExecutions, 1);
    EXPECT_FALSE(pred.statisticBooleanEvaluation(1, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_GT(stat->computationTime.at(0), 0);
    EXPECT_GT(stat->totalComputationTime, 0);
    EXPECT_GE(stat->numSatisfaction, 0);
    EXPECT_EQ(stat->numExecutions, 2);
    stat = {std::make_shared<PredicateStatistics>()};
    EXPECT_EQ(stat->computationTime.size(), 0);
    EXPECT_EQ(stat->totalComputationTime, 0);
    EXPECT_EQ(stat->numSatisfaction, 0);
    EXPECT_EQ(stat->numExecutions, 0);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, GetEvaluationTimer) {
    auto timer{std::make_shared<Timer>()};
    auto stat{std::make_shared<PredicateStatistics>()};
    EXPECT_EQ(timer->getTotalTime(), 0);
    EXPECT_TRUE(pred.statisticBooleanEvaluation(0, world, obstacleOne, timer, stat, obstacleTwo));
    EXPECT_GT(timer->getTotalTime(), 0);
}

TEST_F(KeepsSafeDistancePrecPredicateTest, IsVehicleDependent) { EXPECT_TRUE(pred.isVehicleDependent()); }

TEST_F(KeepsSafeDistancePrecPredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_Augmentation-1_1_S-3";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    std::shared_ptr<State> currentState = std::make_shared<State>(0, 100, 0, 10, 0, 0);
    std::shared_ptr<State> firstState = std::make_shared<State>(1, 40, 0, 50, 0, 0);
    state_map_t trajectoryPrediction{std::pair<size_t, std::shared_ptr<State>>(1, firstState)};
    std::shared_ptr<Obstacle> dynamicObstacle = std::make_shared<Obstacle>(Obstacle(
        1, ObstacleRole::DYNAMIC, currentState, ObstacleType::car, 50, 10, 3, -10, 0.3, trajectoryPrediction, 5, 2));

    auto obstacles{scenarioXml.obstacles};
    obstacles.push_back(dynamicObstacle);

    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, obstacles, {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(100)};
    auto obs2{world->findObstacle(101)};
    auto obs3{world->findObstacle(102)};

    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, obs1, {"0.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(0, world, ego, obs2, {"0.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(21, world, ego, obs1, {"0.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(25, world, ego, obs1, {"0.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(25, world, ego, obs2, {"0.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(25, world, ego, obs3, {"0.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(27, world, ego, obs1, {"0.0"}, true));
    EXPECT_TRUE(pred.booleanEvaluation(30, world, ego, obs1, {"0.0"}, true)); // collision must be checked separately
    EXPECT_TRUE(pred.booleanEvaluation(0, world, dynamicObstacle, obs1, {"0.0"}, true));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, dynamicObstacle, obs1, {"0.0"}, true));
}
