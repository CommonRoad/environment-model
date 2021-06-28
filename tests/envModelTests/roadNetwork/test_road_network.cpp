//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
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
    EXPECT_EQ(roadNetwork->getLanes().size(), 4);
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

TEST_F(RoadNetworkTest, FindLaneByShape) {
    EXPECT_EQ(RoadNetwork::findLaneByShape(roadNetwork->getLanes(), polygonOne)->getId(), 16);
    EXPECT_EQ(RoadNetwork::findLaneByShape(roadNetwork->getLanes(), polygonOne)->getId(), 16);
    EXPECT_EQ(RoadNetwork::findLaneByShape(roadNetwork->getLanes(), polygonTwo)->getId(), 16);
}

TEST_F(RoadNetworkTest, CreateLanesInterstate) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/DEU_TestOvertakingExitRamp-1_1_T-1.xml"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_EQ(roadNetworkScenarioOne->getLanes().size(), 3);
    EXPECT_EQ(roadNetworkScenarioOne->getLanes().at(0)->getContainedLanelets().size(), 9);
    EXPECT_EQ(roadNetworkScenarioOne->getLanes().at(1)->getContainedLanelets().size(), 10);
    EXPECT_EQ(roadNetworkScenarioOne->getLanes().at(2)->getContainedLanelets().size(), 10);

    std::string pathToTestFileTwo{TestUtils::getTestScenarioDirectory() + "/DEU_test_safe_distance.xml"};
    const auto &[obstaclesScenarioTwo, roadNetworkScenarioTwo] = CommandLine::getDataFromCommonRoad(pathToTestFileTwo);
    EXPECT_EQ(roadNetworkScenarioTwo->getLanes().size(), 5);

    std::string pathToTestFileThree{TestUtils::getTestScenarioDirectory() + "/DEU_Muc-2_1_T-1.xml"};
    const auto &[obstaclesScenarioThree, roadNetworkScenarioThree] =
        CommandLine::getDataFromCommonRoad(pathToTestFileThree);
    EXPECT_EQ(roadNetworkScenarioThree->getLanes().size(), 2);

    std::string pathToTestFileFour{TestUtils::getTestScenarioDirectory() +
                                   "/DEU_test_consider_entering_vehicles_for_lane_change.xml"};
    const auto &[obstaclesScenarioFour, roadNetworkScenarioFour] =
        CommandLine::getDataFromCommonRoad(pathToTestFileFour);
    EXPECT_EQ(roadNetworkScenarioFour->getLanes().size(), 3);
    EXPECT_EQ(roadNetworkScenarioFour->getLanes().at(0)->getContainedLanelets().size(), 9);
    EXPECT_EQ(roadNetworkScenarioFour->getLanes().at(1)->getContainedLanelets().size(), 10);
    EXPECT_EQ(roadNetworkScenarioFour->getLanes().at(2)->getContainedLanelets().size(), 10);
}

TEST_F(RoadNetworkTest, CreateLanesUrban) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_EQ(roadNetworkScenarioOne->getLanes().size(), 37);

    std::string pathToTestFileTwo{TestUtils::getTestScenarioDirectory() + "/DEU_TrafficLightTest-1_1_T-1.xml"};
    const auto &[obstaclesScenarioTwo, roadNetworkScenarioTwo] = CommandLine::getDataFromCommonRoad(pathToTestFileTwo);
    EXPECT_EQ(roadNetworkScenarioTwo->getLanes().size(), 16);
}