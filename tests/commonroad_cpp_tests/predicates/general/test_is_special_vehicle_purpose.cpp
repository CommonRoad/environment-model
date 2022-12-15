//
// Created by valentin on 02.11.22.
//

#include "test_is_special_vehicle_purpose.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/obstacle/state.h"

void IsSpecialVehiclePurposeTest::SetUp() {
    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 10, 0, 10, 0, 0, 0, 10, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 20, 2, 10, 0, 0, 0, 20, 2);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                                         std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                                         std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network()};

    world = std::make_shared<World>(World(0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(IsSpecialVehiclePurposeTest, DetectsBus) {
    egoVehicle->setObstacleType(ObstacleType::bus);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
}

TEST_F(IsSpecialVehiclePurposeTest, DetectsTaxi) {
    egoVehicle->setObstacleType(ObstacleType::taxi);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
}

TEST_F(IsSpecialVehiclePurposeTest, DetectsBicycle) {
    egoVehicle->setObstacleType(ObstacleType::bicycle);
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));
}

TEST_F(IsSpecialVehiclePurposeTest, DetectsNonSpecialVehicle) {
    egoVehicle->setObstacleType(ObstacleType::car);
    EXPECT_FALSE(pred.booleanEvaluation(0, world, egoVehicle));
}

TEST_F(IsSpecialVehiclePurposeTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(IsSpecialVehiclePurposeTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
