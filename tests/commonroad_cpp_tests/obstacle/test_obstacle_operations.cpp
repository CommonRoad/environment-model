//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_obstacle_operations.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"

void ObstacleOperationsTest::SetUp() {
    setUpLanelets();
    setUpLane();
    setUpRoadNetwork();
    setUpStates();
    setUpObstacles();
}

TEST_F(ObstacleOperationsTest, GetObstacleById) {
    EXPECT_EQ(obstacle_operations::getObstacleById(obstacleList, 1)->getId(), 1);
    EXPECT_EQ(obstacle_operations::getObstacleById(obstacleList, 2)->getId(), 2);
}

TEST_F(ObstacleOperationsTest, MatchObstacleTypeToString) {
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("car"), ObstacleType::car);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("truck"), ObstacleType::truck);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("pedestrian"), ObstacleType::pedestrian);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("bus"), ObstacleType::bus);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("unknown"), ObstacleType::unknown);
    EXPECT_EQ(obstacle_operations::matchStringToObstacleType("vehicle"), ObstacleType::vehicle);
}

TEST_F(ObstacleOperationsTest, ObstacleDirectlyLeft) {}

TEST_F(ObstacleOperationsTest, ObstaclesLeft) {}

TEST_F(ObstacleOperationsTest, ObstaclesAdjacent) {}

TEST_F(ObstacleOperationsTest, ObstacleDirectlyRight) {}

TEST_F(ObstacleOperationsTest, ObstaclesRight) {}

TEST_F(ObstacleOperationsTest, LaneletsRightOfObstacle) {}

TEST_F(ObstacleOperationsTest, LaneletsLeftOfObstacle) {}