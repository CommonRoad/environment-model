//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
#include "../../interfaces/utility_functions.h"

void LaneletOperationsTest::SetUp() {
    setUpLane();
    setUpRoadNetwork();
}

TEST_F(LaneletOperationsTest, MatchStringToLaneletType) {
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("interstate"), LaneletType::interstate);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("urban"), LaneletType::urban);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("crosswalk"), LaneletType::crosswalk);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("busStop"), LaneletType::busStop);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("country"), LaneletType::country);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("highway"), LaneletType::highway);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("driveWay"), LaneletType::driveWay);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("mainCarriageWay"), LaneletType::mainCarriageWay);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("accessRamp"), LaneletType::accessRamp);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("exitRamp"), LaneletType::exitRamp);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("shoulder"), LaneletType::shoulder);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("bikeLane"), LaneletType::bikeLane);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("sidewalk"), LaneletType::sidewalk);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("busLane"), LaneletType::busLane);
    EXPECT_EQ(lanelet_operations::matchStringToLaneletType("test"), LaneletType::unknown);
}

TEST_F(LaneletOperationsTest, MatchStringToLineMarking) {
    EXPECT_EQ(lanelet_operations::matchStringToLineMarking("solid"), LineMarking::solid);
    EXPECT_EQ(lanelet_operations::matchStringToLineMarking("dashed"), LineMarking::dashed);
    EXPECT_EQ(lanelet_operations::matchStringToLineMarking("broad_solid"), LineMarking::broad_solid);
    EXPECT_EQ(lanelet_operations::matchStringToLineMarking("broad_dashed"), LineMarking::broad_dashed);
    EXPECT_EQ(lanelet_operations::matchStringToLineMarking("no_marking"), LineMarking::no_marking);
    EXPECT_EQ(lanelet_operations::matchStringToLineMarking("test"), LineMarking::unknown);
}

//TEST_F(LaneletOperationsTest, CombineLaneletAndSuccessorsWithSameTypeToLane) {
//    compareVerticesVector(lanelet_operations::combineLaneletAndSuccessorsWithSameTypeToLane(laneletThree).at(0)->getCenterVertices(),
//                          roadNetwork->getLanes().at(0)->getCenterVertices());
//    compareVerticesVector(lanelet_operations::combineLaneletAndSuccessorsWithSameTypeToLane(laneletThree).at(1)->getCenterVertices(),
//                          roadNetwork->getLanes().at(1)->getCenterVertices());
//}

TEST_F(RoadNetworkTest, CreateInterstateLanes) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/DEU_TestOvertakingExitRamp-1_1_T-1.xml"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    auto lanes{lanelet_operations::createInterstateLanes(roadNetworkScenarioOne->getLaneletNetwork(), 1234)};
    EXPECT_EQ(lanes.size(), 3);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 9);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(2)->getContainedLanelets().size(), 10);

    std::string pathToTestFileTwo{TestUtils::getTestScenarioDirectory() + "/DEU_test_safe_distance.xml"};
    const auto &[obstaclesScenarioTwo, roadNetworkScenarioTwo] = CommandLine::getDataFromCommonRoad(pathToTestFileTwo);
    lanes = lanelet_operations::createInterstateLanes(roadNetworkScenarioTwo->getLaneletNetwork(), 1234);
    EXPECT_EQ(lanes.size(), 5);

    std::string pathToTestFileThree{TestUtils::getTestScenarioDirectory() + "/DEU_Muc-2_1_T-1.xml"};
    const auto &[obstaclesScenarioThree, roadNetworkScenarioThree] =
    CommandLine::getDataFromCommonRoad(pathToTestFileThree);
    lanes = lanelet_operations::createInterstateLanes(roadNetworkScenarioThree->getLaneletNetwork(), 1234);
    EXPECT_EQ(lanes.size(), 2);

    std::string pathToTestFileFour{TestUtils::getTestScenarioDirectory() +
        "/DEU_test_consider_entering_vehicles_for_lane_change.xml"};
    const auto &[obstaclesScenarioFour, roadNetworkScenarioFour] =
    CommandLine::getDataFromCommonRoad(pathToTestFileFour);
    lanes = lanelet_operations::createInterstateLanes(roadNetworkScenarioFour->getLaneletNetwork(), 1234);
    EXPECT_EQ(lanes.size(), 3);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 9);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(2)->getContainedLanelets().size(), 10);
}

TEST_F(RoadNetworkTest, CreateLanesByLanelets) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne] = CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    auto lanes{lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioOne->getLaneletNetwork().at(0)}, 1234)};
    EXPECT_EQ(lanes.size(), 1);

    std::string pathToTestFileTwo{TestUtils::getTestScenarioDirectory() + "/DEU_TrafficLightTest-1_1_T-1.xml"};
    const auto &[obstaclesScenarioTwo, roadNetworkScenarioTwo] = CommandLine::getDataFromCommonRoad(pathToTestFileTwo);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioTwo->findLaneletById(10)}, 1234);
    EXPECT_EQ(lanes.size(), 3);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioTwo->findLaneletById(4)}, 1234);
    EXPECT_EQ(lanes.size(), 1);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioTwo->findLaneletById(2)}, 1234);
    EXPECT_EQ(lanes.size(), 1);
}