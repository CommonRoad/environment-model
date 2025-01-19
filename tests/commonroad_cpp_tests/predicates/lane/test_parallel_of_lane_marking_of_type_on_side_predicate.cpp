#include "test_parallel_of_lane_marking_of_type_on_side_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/state.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void ParallelOfLaneMarkingOfTypeOnSidePredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 2, 10, 0, 0, 0, 10, 2);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 6, 10, 0, 0, 0, 20, 6);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 10, 10, 0, 0, 0, 30, 10);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 40, 14, 10, 0, 0, 0, 40, 14);
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 50, 22, 10, 0, 0, 0, 50, 22);
    std::shared_ptr<State> stateSixEgoVehicle = std::make_shared<State>(6, 60, 24, 10, 0, 0, 0, 60, 24);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(5, stateFiveEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(6, stateSixEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_3()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
}

// See adding one more lane to the network

TEST_F(ParallelOfLaneMarkingOfTypeOnSidePredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}, {"left", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle, {}, {"left", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, {}, {"left", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, {"left", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, {"left", "broad"}));
    EXPECT_TRUE(pred.booleanEvaluation(5, world, egoVehicle, {}, {"left", "broad"}));
    EXPECT_TRUE(pred.booleanEvaluation(6, world, egoVehicle, {}, {"left", "broad"}));
}

void ParallelOfLaneMarkingOfTypeOnSidePredicateTest::SetUpRightSide() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 2, 10, 0, 0, 0, 10, 2);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 6, 10, 0, 0, 0, 20, 6);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 30, 10, 10, 0, 0, 0, 30, 10);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 40, 14, 10, 0, 0, 0, 40, 14);
    std::shared_ptr<State> stateFiveEgoVehicle = std::make_shared<State>(5, 50, 22, 10, 0, 0, 0, 50, 22);
    std::shared_ptr<State> stateSixEgoVehicle = std::make_shared<State>(6, 60, 24, 10, 0, 0, 0, 60, 24);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(5, stateFiveEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(6, stateSixEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_3()};

    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(ParallelOfLaneMarkingOfTypeOnSidePredicateTest, BooleanEvaluationObjectsRightSide) {
    SetUpRightSide();
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle, {}, {"right", "broad"}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}, {"right", "broad"}));
    EXPECT_TRUE(pred.booleanEvaluation(2, world, egoVehicle, {}, {"right", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}, {"right", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}, {"right", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(5, world, egoVehicle, {}, {"right", "broad"}));
    EXPECT_FALSE(pred.booleanEvaluation(6, world, egoVehicle, {}, {"right", "broad"}));
}

TEST_F(ParallelOfLaneMarkingOfTypeOnSidePredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(ParallelOfLaneMarkingOfTypeOnSidePredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
