#include "test_close_to_intersection_predicate.h"

#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void CloseToIntersectionPredicateTest::SetUp() {
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 55, 2, 30, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 85, 2, 30, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 115, 2, 30, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 145, 2, 30, 0, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 175, 2, 30, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
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
    this->world = std::make_shared<World>(World(0, roadNetwork, {this->egoVehicle}, {}, 0.1));
}

TEST_F(CloseToIntersectionPredicateTest, OnIncoming) {
    initializeTestData(LaneletType::incoming, LaneletType::urban, LaneletType::intersectionRightTurn,
                       LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(4, world, egoVehicle, {}));
}

TEST_F(CloseToIntersectionPredicateTest, SuccessorIsIncoming) {
    initializeTestData(LaneletType::urban, LaneletType::urban, LaneletType::incoming, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle, {}));
}

TEST_F(CloseToIntersectionPredicateTest, TwoIncomingsInARow) {
    initializeTestData(LaneletType::incoming, LaneletType::urban, LaneletType::incoming, LaneletType::intersection);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle, {}));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle, {}));
    EXPECT_FALSE(pred.booleanEvaluation(3, world, egoVehicle, {}));
    EXPECT_TRUE(pred.booleanEvaluation(4, world, egoVehicle, {}));
}
