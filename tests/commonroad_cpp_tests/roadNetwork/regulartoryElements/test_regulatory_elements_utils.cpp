//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_regulatory_elements_utils.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"

void RegulatoryElementsUtilsTest::SetUp() {
    //    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 45, 0, 0, 0, 0, 0);
    //    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 45, 2, 50, 0, 0, 0, 45, 0);
    //    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 95, 2, 55, 0, 0, 0, 95, 0);
    //    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 150, 2, 45, 0, 0, 0, 150, 0);
    //
    //    std::map<size_t, std::shared_ptr<State>> trajectoryPredictionEgoVehicle{
    //        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
    //        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
    //        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
    //        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};
    //
    //    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, false, stateZeroObstacleOne, ObstacleType::car, 50, 10,
    //    3, -10,
    //                                                      0.3, trajectoryPredictionEgoVehicle, 5, 2));
    //
    //    auto roadNetwork{utils_predicate_test::create_road_network_2()};
    //
    //    world =
    //        std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {},
    //        0.1));
}

TEST_F(RegulatoryElementsUtilsTest, TypeSpeedLimit) {
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::truck), 22.22);
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::pedestrian), std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::car), std::numeric_limits<double>::max());
}