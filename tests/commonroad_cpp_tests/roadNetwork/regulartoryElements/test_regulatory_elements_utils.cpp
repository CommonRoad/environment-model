//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_regulatory_elements_utils.h"
#include "../../predicates/utils_predicate_test.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"

void RegulatoryElementsUtilsTest::SetUp() {
    std::shared_ptr<State> stateZeroObstacleOne = std::make_shared<State>(0, 0, 2, 45, 0, 0, 0, 0, 0);
    std::shared_ptr<State> stateOneObstacleOne = std::make_shared<State>(1, 45, 2, 50, 0, 0, 0, 45, 0);
    std::shared_ptr<State> stateTwoObstacleOne = std::make_shared<State>(2, 95, 2, 55, 0, 0, 0, 95, 0);
    std::shared_ptr<State> stateThreeObstacleOne = std::make_shared<State>(3, 150, 2, 45, 0, 0, 0, 150, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleOne),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleOne),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleOne),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleOne)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroObstacleOne, ObstacleType::car,
                                                      50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    auto roadNetwork{utils_predicate_test::create_road_network_2()};
    world =
        std::make_shared<World>(World(0, roadNetwork, std::vector<std::shared_ptr<Obstacle>>{obstacleOne}, {}, 0.1));
}

TEST_F(RegulatoryElementsUtilsTest, TypeSpeedLimit) {
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::truck), 22.22);
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::pedestrian), std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::typeSpeedLimit(ObstacleType::car), std::numeric_limits<double>::max());
}

TEST_F(RegulatoryElementsUtilsTest, SpeedLimitSingle) {
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(111), "274"), 35.0);
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(222), "274"),
              std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(333), "274"),
              std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::speedLimit(world->getRoadNetwork()->findLaneletById(444), "274"),
              std::numeric_limits<double>::max());
}

TEST_F(RegulatoryElementsUtilsTest, SpeedLimitVector) {
    EXPECT_EQ(
        regulatory_elements_utils::speedLimit(
            {world->getRoadNetwork()->findLaneletById(111), world->getRoadNetwork()->findLaneletById(222)}, "274"),
        35.0);
    EXPECT_EQ(
        regulatory_elements_utils::speedLimit(
            {world->getRoadNetwork()->findLaneletById(222), world->getRoadNetwork()->findLaneletById(444)}, "274"),
        std::numeric_limits<double>::max());
    EXPECT_EQ(
        regulatory_elements_utils::speedLimit(
            {world->getRoadNetwork()->findLaneletById(333), world->getRoadNetwork()->findLaneletById(222)}, "274"),
        std::numeric_limits<double>::max());
    EXPECT_EQ(regulatory_elements_utils::speedLimit({world->getRoadNetwork()->findLaneletById(111)}, "274"), 35.0);
}

TEST_F(RegulatoryElementsUtilsTest, RequiredVelocitySingle) {
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(111), "275"), 10.0);
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(222), "275"), 0);
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(333), "275"), 0);
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity(world->getRoadNetwork()->findLaneletById(444), "275"), 0);
}

TEST_F(RegulatoryElementsUtilsTest, RequiredVelocityVector) {
    EXPECT_EQ(
        regulatory_elements_utils::requiredVelocity(
            {world->getRoadNetwork()->findLaneletById(111), world->getRoadNetwork()->findLaneletById(222)}, "275"),
        10.0);
    EXPECT_EQ(
        regulatory_elements_utils::requiredVelocity(
            {world->getRoadNetwork()->findLaneletById(222), world->getRoadNetwork()->findLaneletById(444)}, "275"),
        0);
    EXPECT_EQ(
        regulatory_elements_utils::requiredVelocity(
            {world->getRoadNetwork()->findLaneletById(333), world->getRoadNetwork()->findLaneletById(222)}, "275"),
        0);
    EXPECT_EQ(regulatory_elements_utils::requiredVelocity({world->getRoadNetwork()->findLaneletById(111)}, "275"),
              10.0);
}

TEST_F(RegulatoryElementsUtilsTest, SpeedLimitSuggested) {
    EXPECT_EQ(
        regulatory_elements_utils::speedLimitSuggested(
            {world->getRoadNetwork()->findLaneletById(111), world->getRoadNetwork()->findLaneletById(222)}, "274"),
        35.0);
    EXPECT_EQ(
        regulatory_elements_utils::speedLimitSuggested(
            {world->getRoadNetwork()->findLaneletById(222), world->getRoadNetwork()->findLaneletById(444)}, "274"),
        36.11);
    EXPECT_EQ(
        regulatory_elements_utils::speedLimitSuggested(
            {world->getRoadNetwork()->findLaneletById(333), world->getRoadNetwork()->findLaneletById(222)}, "274"),
        36.11);
    EXPECT_EQ(regulatory_elements_utils::speedLimitSuggested({world->getRoadNetwork()->findLaneletById(111)}, "274"),
              35.0);
}

TEST_F(RegulatoryElementsUtilsTest, TrafficSignReferencesStopSign) {}

TEST_F(RegulatoryElementsUtilsTest, AtRedTrafficLight) {}

TEST_F(RegulatoryElementsUtilsTest, ActiveTrafficLights) {}

// TEST_F(RegulatoryElementsUtilsTest, TrafficSignInFront) {
//    EXPECT_TRUE(regulatory_elements_utils::trafficSignInFront(
//        0, world->getRoadNetwork()->findLaneletById(111)->getTrafficSigns().at(0), obstacleOne,
//        world->getRoadNetwork()));
//    EXPECT_FALSE(regulatory_elements_utils::trafficSignInFront(
//        1, world->getRoadNetwork()->findLaneletById(111)->getTrafficSigns().at(0), obstacleOne,
//        world->getRoadNetwork()));
//}