//
// Created by sebastian on 25.12.20.
//

#include "test_obstacle_operations.h"
#include "obstacle/obstacle_operations.h"

void ObstacleOperationsTest::SetUp(){
    setUpLanelets();
    setUpLane();
    setUpRoadNetwork();
    setUpStates();
    setUpObstacles();
}

TEST_F(ObstacleOperationsTest, GetObstacleById){
    EXPECT_EQ(getObstacleById(obstacleList, 1)->getId(), 1);
    EXPECT_EQ(getObstacleById(obstacleList, 2)->getId(), 2);
}

TEST_F(ObstacleOperationsTest, MatchObstacleTypeToString){
    EXPECT_EQ(matchObstacleTypeToString("car"), ObstacleType::car);
    EXPECT_EQ(matchObstacleTypeToString("truck"), ObstacleType::truck);
    EXPECT_EQ(matchObstacleTypeToString("pedestrian"), ObstacleType::pedestrian);
    EXPECT_EQ(matchObstacleTypeToString("bus"), ObstacleType::bus);
    EXPECT_EQ(matchObstacleTypeToString("unknown"), ObstacleType::unknown);
    EXPECT_EQ(matchObstacleTypeToString("vehicle"), ObstacleType::vehicle);
}