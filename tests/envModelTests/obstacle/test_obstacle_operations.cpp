//
// Created by sebastian on 25.12.20.
//

#include "test_obstacle_operations.h"
#include "obstacle/obstacle_operations.h"

void ObstacleOperationsTest::SetUp() {
    setUpLanelets();
    setUpLane();
    setUpRoadNetwork();
    setUpStates();
    setUpObstacles();
}

TEST_F(ObstacleOperationsTest, GetObstacleById) {
    EXPECT_EQ(getObstacleById(obstacleList, 1)->getId(), 1);
    EXPECT_EQ(getObstacleById(obstacleList, 2)->getId(), 2);
}

TEST_F(ObstacleOperationsTest, MatchObstacleTypeToString) {
    EXPECT_EQ(matchStringToObstacleType("car"), ObstacleType::car);
    EXPECT_EQ(matchStringToObstacleType("truck"), ObstacleType::truck);
    EXPECT_EQ(matchStringToObstacleType("pedestrian"), ObstacleType::pedestrian);
    EXPECT_EQ(matchStringToObstacleType("bus"), ObstacleType::bus);
    EXPECT_EQ(matchStringToObstacleType("unknown"), ObstacleType::unknown);
    EXPECT_EQ(matchStringToObstacleType("vehicle"), ObstacleType::vehicle);
}