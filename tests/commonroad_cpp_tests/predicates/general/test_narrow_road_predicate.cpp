#include "test_narrow_road_predicate.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"
#include <cmath>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>

void NarrowRoadPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 10, 1, 10, 0, 0, 0, 10, 1);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 2);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                                         std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                                         std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};
    world = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {}, 0.1));

    auto roadNetwork2{utils_predicate_test::create_narrow_road_network(5.5)};
    world2 = std::make_shared<World>(World(0, roadNetwork2, {egoVehicle}, {}, 0.1));

    auto roadNetwork3{utils_predicate_test::create_narrow_road_network(7)};
    world3 = std::make_shared<World>(World(0, roadNetwork3, {egoVehicle}, {}, 0.1));

    auto roadNetwork4{utils_predicate_test::create_narrow_road_network(3)};
    world4 = std::make_shared<World>(World(0, roadNetwork4, {egoVehicle}, {}, 0.1));

    std::string pathToTestFileOncoming =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_two_lanes_with_opposite_directions.xml";
    const auto &[obstaclesOncoming, roadNetworkOncoming, timeStepSizeOncoming] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncoming);
    worldOncoming = std::make_shared<World>(World(0, roadNetworkOncoming, {egoVehicle}, {}, 0.1));

    std::string pathToTestFileOncomingNarrow =
        TestUtils::getTestScenarioDirectory() + "/predicates/DEU_two_lanes_with_opposite_directions_narrow.xml";
    const auto &[obstaclesOncomingNarrow, roadNetworkOncomingNarrow, timeStepSizeOncomingNarrow] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOncomingNarrow);
    worldOncomingNarrow = std::make_shared<World>(World(0, roadNetworkOncomingNarrow, {egoVehicle}, {}, 0.1));
}

TEST_F(NarrowRoadPredicateTest, BooleanEvaluationObjects) {
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle));

    EXPECT_TRUE(pred.booleanEvaluation(0, world2, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, world2, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world2, egoVehicle));

    EXPECT_FALSE(pred.booleanEvaluation(0, world3, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, world3, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, world3, egoVehicle));

    EXPECT_TRUE(pred.booleanEvaluation(0, world4, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, world4, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, world4, egoVehicle));

    EXPECT_TRUE(pred.booleanEvaluation(0, worldOncomingNarrow, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(1, worldOncomingNarrow, egoVehicle));
    EXPECT_TRUE(pred.booleanEvaluation(2, worldOncomingNarrow, egoVehicle));
    //
    EXPECT_FALSE(pred.booleanEvaluation(0, worldOncoming, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(1, worldOncoming, egoVehicle));
    EXPECT_FALSE(pred.booleanEvaluation(2, worldOncoming, egoVehicle));
}

TEST_F(NarrowRoadPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(NarrowRoadPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}