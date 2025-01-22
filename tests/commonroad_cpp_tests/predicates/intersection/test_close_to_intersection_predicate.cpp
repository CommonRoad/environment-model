#include "test_close_to_intersection_predicate.h"

#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void CloseToIntersectionPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 55, 2, 30, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 85, 2, 30, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 115, 2, 30, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 145, 2, 30, 0, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 175, 2, 30, 0, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
}

void CloseToIntersectionPredicateTest::initializeTestData(LaneletType laneletTypeRight, LaneletType laneletTypeLeft,
                                                          LaneletType laneletTypeSuccessorRight,
                                                          LaneletType laneletTypeSuccessorLeft) {
    auto roadNetwork{utils_predicate_test::create_road_network_with_2_successors(
        {laneletTypeRight}, {laneletTypeLeft}, {laneletTypeSuccessorRight}, {laneletTypeSuccessorLeft})};
    this->world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {this->egoVehicle}, {}, 0.1));
}

TEST_F(CloseToIntersectionPredicateTest, OnIncoming) {
    initializeTestData(LaneletType::incoming, LaneletType::urban, LaneletType::right, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, {"30"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, {"30"}));
}

TEST_F(CloseToIntersectionPredicateTest, SuccessorIsIncoming) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::incoming, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, {"30"}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle, {}, {"30"}));
}

TEST_F(CloseToIntersectionPredicateTest, TwoIncomingsInARow) {
    initializeTestData(LaneletType::incoming, LaneletType::urban, LaneletType::incoming, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, {"30"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, {}, {"30"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, {"30"}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle, {}, {"30"}));
}

TEST_F(CloseToIntersectionPredicateTest, InFrontOfIntersection) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_Intersection.xml"};
    const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);

    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 4.0, 0.0, 0, 0, 0, 0, 4, 0.0);
    std::shared_ptr<State> stateZeroObstacleTwo = std::make_shared<State>(0, 16.0, 0, 0, 0, 0, 0, 16, 0.0);
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 33.0, 3, 0, 0, M_PI, 0, 33.0, 0.0);

    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 17.0, 0.0, 0, 0, 0, 0, 17, 0.0);
    std::shared_ptr<State> stateOneObstacleTwo = std::make_shared<State>(1, 10, 3, 0, 0, 0, 0, 10, 0.0);

    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 17.4, 0, 0, 0, 0, 0, 17.4, 0.0);
    std::shared_ptr<State> stateTwoObstacleTwo = std::make_shared<State>(2, 23, 0, 0, 0, 0, 0, 23, 0.0);

    state_map_t trajectoryPredictionObstacleOne{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne)};

    state_map_t trajectoryPredictionObstacleTwo{std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleTwo),
                                                std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleTwo)};

    auto obstacleOne{
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleOne, 5, 2))};
    auto obstacleTwo{
        std::make_shared<Obstacle>(Obstacle(2, ObstacleRole::DYNAMIC, stateZeroObstacleTwo, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleTwo, 5, 2))};

    auto obstacleThree{std::make_shared<Obstacle>(
        Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10, 3, -10, 0.3, {}, 5, 2))};

    auto worldNew{std::make_shared<World>(
        World("testWorld", 0, roadNetwork,
              std::vector<std::shared_ptr<Obstacle>>{obstacleOne, obstacleTwo, obstacleThree}, {}, timeStepSize))};

    EXPECT_FALSE(pred.booleanEvaluation(0, worldNew, obstacleOne, {}, {"1.5"}));  // far away
    EXPECT_FALSE(pred.booleanEvaluation(0, worldNew, obstacleTwo, {}, {"1.5"}));  // 2.5m from intersection away
    EXPECT_TRUE(pred.booleanEvaluation(0, worldNew, obstacleThree, {}, {"1.5"})); // 0.5m away from intersection

    EXPECT_TRUE(pred.booleanEvaluation(1, worldNew, obstacleOne, {}, {"1.5"})); // 0.5 from intersection away
    EXPECT_FALSE(
        pred.booleanEvaluation(1, worldNew, obstacleTwo, {}, {"1.5"})); // too far away from intersection in opp dir

    EXPECT_TRUE(pred.booleanEvaluation(2, worldNew, obstacleOne, {}, {"1.5"}));  // 0.1m away from intersection
    EXPECT_FALSE(pred.booleanEvaluation(2, worldNew, obstacleTwo, {}, {"1.5"})); // already in intersection
}

TEST_F(CloseToIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(CloseToIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
