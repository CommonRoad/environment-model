//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_road_network.h"

void RoadNetworkTestInitialization::setUpRoadNetwork() {
    std::vector<std::shared_ptr<Lanelet>> lanelets{laneletOne, laneletTwo, laneletThree, laneletFour, laneletFive};

    roadNetwork = std::make_shared<RoadNetwork>(RoadNetwork(lanelets));
    // TODO add intersection
}

void RoadNetworkTest::SetUp() {
    setUpLane();
    setUpRoadNetwork();
}

TEST_F(RoadNetworkTest, InitializationComplete) {
    EXPECT_EQ(roadNetwork->getLaneletNetwork().size(), 5);
    EXPECT_EQ(roadNetwork->getLaneletNetwork().at(0)->getId(), 1);
}

TEST_F(RoadNetworkTest, FindOccupiedLaneletsByShape) {
    EXPECT_EQ(roadNetwork->findOccupiedLaneletsByShape(polygonOne).size(), 2); // order can be random
    EXPECT_EQ(roadNetwork->findOccupiedLaneletsByShape(polygonTwo).at(0)->getId(), 1);
    EXPECT_EQ(roadNetwork->findOccupiedLaneletsByShape(polygonThree).size(), 0);
}

TEST_F(RoadNetworkTest, FindLaneletsByPosition) {
    EXPECT_EQ(roadNetwork->findLaneletsByPosition(1, 0.5).at(0)->getId(), 1);
    EXPECT_EQ(roadNetwork->findLaneletsByPosition(123, 123).size(), 0);
}

TEST_F(RoadNetworkTest, FindLaneletById) {
    EXPECT_EQ(roadNetwork->findLaneletById(1)->getId(), 1);
    EXPECT_THROW(roadNetwork->findLaneletById(123)->getId(), std::domain_error);
}
