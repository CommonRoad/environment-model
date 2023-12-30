#include "test_on_lanelet_with_successor_type_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void OnLaneletWithSuccessorTypePredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 2, 10, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 4, 10, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 6, 10, 0, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 40, 8, 10, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
        std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
    };

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
    // To test that the first timestep with a distance to the successor lanelet of 100m evaluates to false
    egoVehicle->setFieldOfViewFrontDistance(90.1);
}

void OnLaneletWithSuccessorTypePredicateTest::initializeTestData(LaneletType laneletTypeRight,
                                                                 LaneletType laneletTypeLeft,
                                                                 LaneletType laneletTypeSuccessorRight,
                                                                 LaneletType laneletTypeSuccessorLeft) {
    auto roadNetwork{utils_predicate_test::create_road_network_with_2_successors(
        {laneletTypeRight}, {laneletTypeLeft}, {laneletTypeSuccessorRight}, {laneletTypeSuccessorLeft})};
    this->world = std::make_shared<World>(World(0, roadNetwork, {this->egoVehicle}, {}, 0.1));
    opt = std::make_shared<OptionalPredicateParameters>();
    opt->laneletType = {laneletTypeSuccessorRight};
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnIntersectionRightGoingLaneNext) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::right, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnIntersectionLeftGoingLaneNext) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::left, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnIntersectionStraightGoingLaneNext) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::straight, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnShoulderNext) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::shoulder, LaneletType::interstate);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnUrbanRoadNext) {
    initializeTestData(LaneletType::exitRamp, LaneletType::exitRamp, LaneletType::urban, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnMainCarriageWayNext) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::mainCarriageWay, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnExitRampNext) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::exitRamp, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, BooleanEvaluationOnAccessRampNext) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::accessRamp, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, opt));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, opt));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, opt));
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(OnLaneletWithSuccessorTypePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
