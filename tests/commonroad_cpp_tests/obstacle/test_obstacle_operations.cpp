//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_obstacle_operations.h"
#include "commonroad_cpp/geometry/geometric_operations.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"

void ObstacleOperationsTest::SetUp() {
    setUpLanelets();
    setUpLane();
    setUpRoadNetwork();
    setUpStates();
    setUpObstacles();
    for (size_t timeStep{0}; timeStep < 11; ++timeStep)
        obstacleThree->setOccupiedLanes(roadNetwork, timeStep);
    obstacleFour->setOccupiedLanes(roadNetwork, 7);
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

TEST_F(ObstacleOperationsTest, ObstacleDirectlyLeft) {
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(0, obstacleList, obstacleThree, roadNetwork),
              nullptr); // static obstacle adjacent right
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(1, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(2, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(3, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(4, obstacleList, obstacleThree, roadNetwork)->getId(),
              5); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(5, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(6, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(7, obstacleList, obstacleThree, roadNetwork)->getId(),
              5); // two left vehicles
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(8, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(9, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyLeft(10, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially right of in front and behind
}

TEST_F(ObstacleOperationsTest, ObstaclesLeft) {
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(0, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // static obstacle adjacent right
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(1, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle exactly left of
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(1, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(2, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle partially left of in front
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(2, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(3, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle partially left of in behind
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(3, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(4, obstacleList, obstacleThree, roadNetwork).size(),
    //              1); // other vehicle left of in front and behind
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(4, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(5, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle in same lane in front
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(6, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstaclesLeft(7, obstacleList, obstacleThree, roadNetwork).size(),
              2); // other vehicle exactly right of
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(7, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 2);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(7, obstacleList, obstacleThree, roadNetwork).at(1)->getId(), 5);
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(8, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle partially right of in front
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(9, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle partially right of behind
    //    EXPECT_EQ(obstacle_operations::obstaclesLeft(10, obstacleList, obstacleThree, roadNetwork).size(),
    //              0); // other vehicle partially right of in front and behind
}

TEST_F(ObstacleOperationsTest, ObstaclesAdjacent) {
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(0, obstacleList, obstacleThree, roadNetwork).size(),
              1); // static obstacle adjacent
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(0, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 1);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(1, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(1, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(2, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(2, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(3, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(3, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(4, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(4, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(5, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(6, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).size(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 2);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).at(1)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(7, obstacleList, obstacleThree, roadNetwork).at(2)->getId(), 5);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(8, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(8, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(9, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(9, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(10, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesAdjacent(10, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
}

TEST_F(ObstacleOperationsTest, ObstacleDirectlyRight) {
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(0, obstacleList, obstacleThree, roadNetwork),
              nullptr); // static obstacle adjacent right
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(1, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(2, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(3, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(4, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(5, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(6, obstacleList, obstacleThree, roadNetwork),
              nullptr); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(7, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle exactly right of
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(8, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(9, obstacleList, obstacleThree, roadNetwork)->getId(),
              4); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstacleDirectlyRight(10, obstacleList, obstacleThree, roadNetwork)->getId(),
              5); // other vehicle partially right of in front and behind
}

TEST_F(ObstacleOperationsTest, ObstaclesRight) {
    EXPECT_EQ(obstacle_operations::obstaclesRight(0, obstacleList, obstacleThree, roadNetwork).size(),
              0); // static obstacle adjacent right
    EXPECT_EQ(obstacle_operations::obstaclesRight(1, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle exactly left of
    EXPECT_EQ(obstacle_operations::obstaclesRight(2, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle partially left of in front
    EXPECT_EQ(obstacle_operations::obstaclesRight(3, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle partially left of in behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(4, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle left of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(5, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in same lane in front
    EXPECT_EQ(obstacle_operations::obstaclesRight(6, obstacleList, obstacleThree, roadNetwork).size(),
              0); // other vehicle in right lane but not adjacent
    EXPECT_EQ(obstacle_operations::obstaclesRight(7, obstacleList, obstacleThree, roadNetwork).size(),
              2); // other vehicle exactly right of
    EXPECT_EQ(obstacle_operations::obstaclesRight(7, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesRight(7, obstacleList, obstacleThree, roadNetwork).at(1)->getId(), 6);
    EXPECT_EQ(obstacle_operations::obstaclesRight(8, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front
    EXPECT_EQ(obstacle_operations::obstaclesRight(8, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesRight(9, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(9, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 4);
    EXPECT_EQ(obstacle_operations::obstaclesRight(10, obstacleList, obstacleThree, roadNetwork).size(),
              1); // other vehicle partially right of in front and behind
    EXPECT_EQ(obstacle_operations::obstaclesRight(10, obstacleList, obstacleThree, roadNetwork).at(0)->getId(), 5);
}

TEST_F(ObstacleOperationsTest, LaneletsRightOfObstacle) {
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(0, roadNetwork, obstacleThree).size(), 1);
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(1, roadNetwork, obstacleThree).size(), 1);
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(7, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsRightOfObstacle(7, roadNetwork, obstacleThree).size(), 0);
}

TEST_F(ObstacleOperationsTest, LaneletsLeftOfObstacle) {
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(0, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(1, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(7, roadNetwork, obstacleThree).size(), 0);
    EXPECT_EQ(obstacle_operations::laneletsLeftOfObstacle(7, roadNetwork, obstacleFour).size(), 1);
}