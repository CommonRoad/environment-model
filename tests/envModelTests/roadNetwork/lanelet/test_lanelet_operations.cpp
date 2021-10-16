//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "test_lanelet_operations.h"

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

TEST_F(RoadNetworkTest, CreateInterstateLanes) {
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/DEU_TestOvertakingExitRamp-1_1_T-1.xml"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        CommandLine::getDataFromCommonRoad(pathToTestFileOne);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    auto lanes{lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioOne->findLaneletById(3)},
                                                               globalIdRef, roadNetworkScenarioOne)};
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 11);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 9);

    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioOne->findLaneletById(11)}, globalIdRef,
                                                            roadNetworkScenarioOne);
    EXPECT_EQ(lanes.size(), 2);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 11);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 9);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().front()->getId(), 11);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().back()->getId(), 20);

    std::string pathToTestFileTwo{TestUtils::getTestScenarioDirectory() + "/DEU_test_safe_distance.xml"};
    const auto &[obstaclesScenarioTwo, roadNetworkScenarioTwo, timeStepSizeTwo] =
        CommandLine::getDataFromCommonRoad(pathToTestFileTwo);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioTwo->findLaneletById(22)}, globalIdRef,
                                                            roadNetworkScenarioTwo);
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 21);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 30);

    std::string pathToTestFileThree{TestUtils::getTestScenarioDirectory() + "/DEU_Muc-2_1_T-1.xml"};
    const auto &[obstaclesScenarioThree, roadNetworkScenarioThree, timeStepSizeThree] =
        CommandLine::getDataFromCommonRoad(pathToTestFileThree);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioThree->findLaneletById(34782)},
                                                            globalIdRef, roadNetworkScenarioThree);
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 34782);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 34782);

    std::string pathToTestFileFour{TestUtils::getTestScenarioDirectory() +
                                   "/DEU_test_consider_entering_vehicles_for_lane_change.xml"};
    const auto &[obstaclesScenarioFour, roadNetworkScenarioFour, timeStepSizeFour] =
        CommandLine::getDataFromCommonRoad(pathToTestFileFour);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioFour->findLaneletById(3)}, globalIdRef,
                                                            roadNetworkScenarioFour);
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 20);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenarioFour->findLaneletById(20)}, globalIdRef,
                                                            roadNetworkScenarioFour);
    EXPECT_EQ(lanes.size(), 2);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 20);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().size(), 10);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().front()->getId(), 11);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().back()->getId(), 20);
}

TEST_F(RoadNetworkTest, CreateLanesByLaneletsSingleSimpleIntersection) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/DEU_TrafficLightTest-1_1_T-1.xml"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        CommandLine::getDataFromCommonRoad(pathToTestFile);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    auto lanes{lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(10)}, globalIdRef,
                                                               roadNetworkScenario)};
    EXPECT_EQ(lanes.size(), 3);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 3);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().size(), 3);
    EXPECT_EQ(lanes.at(2)->getContainedLanelets().size(), 3);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 10);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().front()->getId(), 10);
    EXPECT_EQ(lanes.at(2)->getContainedLanelets().front()->getId(), 10);
    std::set<size_t> endIds{lanes.at(0)->getContainedLanelets().back()->getId(),
                            lanes.at(1)->getContainedLanelets().back()->getId(),
                            lanes.at(2)->getContainedLanelets().back()->getId()};
    std::set<size_t> expEndIds{2, 7, 15};
    EXPECT_EQ(endIds, expEndIds);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(4)}, globalIdRef,
                                                            roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 3);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 10);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 2);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(2)}, globalIdRef,
                                                            roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 3);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 3);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().size(), 3);
    EXPECT_EQ(lanes.at(2)->getContainedLanelets().size(), 3);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 2);
    EXPECT_EQ(lanes.at(1)->getContainedLanelets().back()->getId(), 2);
    EXPECT_EQ(lanes.at(2)->getContainedLanelets().back()->getId(), 2);
    std::set<size_t> startIds{lanes.at(0)->getContainedLanelets().front()->getId(),
                              lanes.at(1)->getContainedLanelets().front()->getId(),
                              lanes.at(2)->getContainedLanelets().front()->getId()};
    std::set<size_t> expStartIds{10, 8, 16};
    EXPECT_EQ(startIds, expStartIds);
}

TEST_F(RoadNetworkTest, CreateLanesByLaneletsSingleComplexIntersection1) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        CommandLine::getDataFromCommonRoad(pathToTestFile);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    auto lanes{lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(3454)},
                                                               globalIdRef, roadNetworkScenario)};
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 7);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().front()->getId(), 3567);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().back()->getId(), 3467);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(3502)}, globalIdRef,
                                                            roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 2);
    std::set<size_t> expLaneSizes{7};
    std::set<size_t> laneSizes{lanes.at(0)->getContainedLanelets().size(), lanes.at(1)->getContainedLanelets().size()};
    EXPECT_EQ(expLaneSizes, laneSizes);
    auto startIds{std::set<size_t>{lanes.at(0)->getContainedLanelets().front()->getId(),
                                   lanes.at(1)->getContainedLanelets().front()->getId()}};
    auto expStartIds{std::set<size_t>{3502}};
    EXPECT_EQ(startIds, expStartIds);
    auto endIds{std::set<size_t>{lanes.at(0)->getContainedLanelets().back()->getId(),
                                 lanes.at(1)->getContainedLanelets().back()->getId()}};
    auto expEndIds{std::set<size_t>{3470, 3484}};
    EXPECT_EQ(endIds, expEndIds);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(3638)}, globalIdRef,
                                                            roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 2);
    expLaneSizes = {7, 5};
    laneSizes = {lanes.at(0)->getContainedLanelets().size(), lanes.at(1)->getContainedLanelets().size()};
    EXPECT_EQ(expLaneSizes, laneSizes);
    startIds = {lanes.at(0)->getContainedLanelets().front()->getId(),
                lanes.at(1)->getContainedLanelets().front()->getId()};
    expStartIds = {3499, 3561};
    EXPECT_EQ(startIds, expStartIds);
    endIds = {lanes.at(0)->getContainedLanelets().back()->getId(), lanes.at(1)->getContainedLanelets().back()->getId()};
    expEndIds = {3481};
    EXPECT_EQ(endIds, expEndIds);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(3530)}, globalIdRef,
                                                            roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 2);
    expLaneSizes = {5, 7};
    laneSizes = {lanes.at(0)->getContainedLanelets().size(), lanes.at(1)->getContainedLanelets().size()};
    EXPECT_EQ(expLaneSizes, laneSizes);
    startIds = {lanes.at(0)->getContainedLanelets().front()->getId(),
                lanes.at(1)->getContainedLanelets().front()->getId()};
    expStartIds = {3499};
    EXPECT_EQ(startIds, expStartIds);
    endIds = {lanes.at(0)->getContainedLanelets().back()->getId(), lanes.at(1)->getContainedLanelets().back()->getId()};
    expEndIds = {3536, 3481};
    EXPECT_EQ(endIds, expEndIds);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(3671)}, globalIdRef,
                                                            roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.at(0)->getContainedLanelets().size(), 5);
    EXPECT_EQ(3561, lanes.at(0)->getContainedLanelets().front()->getId());
    EXPECT_EQ(3481, lanes.at(0)->getContainedLanelets().back()->getId());
    lanes = lanelet_operations::createLanesBySingleLanelets(
        {roadNetworkScenario->findLaneletById(3664), roadNetworkScenario->findLaneletById(3492),
         roadNetworkScenario->findLaneletById(3680), roadNetworkScenario->findLaneletById(3678),
         roadNetworkScenario->findLaneletById(3495), roadNetworkScenario->findLaneletById(3676)},
        globalIdRef, roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 4);
}

TEST_F(RoadNetworkTest, CreateLanesByLaneletsSingleComplexIntersection2) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/USA_Peach-4_1_T-1.xml"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        CommandLine::getDataFromCommonRoad(pathToTestFile);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    auto lanes{lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(43205)},
                                                               globalIdRef, roadNetworkScenario)};
    EXPECT_EQ(lanes.size(), 2);
}

TEST_F(RoadNetworkTest, CreateLanesByLaneletsSeveralComplexIntersections) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/DEU_Guetersloh-25_4_T-1.xml"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        CommandLine::getDataFromCommonRoad(pathToTestFile);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    auto lanes{lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(82817)},
                                                               globalIdRef, roadNetworkScenario)};
    EXPECT_EQ(lanes.size(), 3);
    std::set<size_t> expLaneSizes{5, 7};
    std::set<size_t> laneSizes{lanes.at(0)->getContainedLanelets().size(), lanes.at(1)->getContainedLanelets().size(),
                               lanes.at(2)->getContainedLanelets().size()};
    EXPECT_EQ(expLaneSizes, laneSizes);
    auto startIds{std::set<size_t>{lanes.at(0)->getContainedLanelets().front()->getId(),
                                   lanes.at(1)->getContainedLanelets().front()->getId(),
                                   lanes.at(2)->getContainedLanelets().front()->getId()}};
    auto expStartIds{std::set<size_t>{77695}};
    EXPECT_EQ(startIds, expStartIds);
    auto endIds{std::set<size_t>{lanes.at(0)->getContainedLanelets().back()->getId(),
                                 lanes.at(1)->getContainedLanelets().back()->getId(),
                                 lanes.at(2)->getContainedLanelets().back()->getId()}};
    auto expEndIds{std::set<size_t>{77071, 77236, 77301}};
    EXPECT_EQ(endIds, expEndIds);
    lanes = lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(77065)}, globalIdRef,
                                                            roadNetworkScenario);
    EXPECT_EQ(lanes.size(), 6);
    expLaneSizes = {5, 7};
    laneSizes = {lanes.at(0)->getContainedLanelets().size(), lanes.at(1)->getContainedLanelets().size(),
                 lanes.at(2)->getContainedLanelets().size(), lanes.at(3)->getContainedLanelets().size(),
                 lanes.at(4)->getContainedLanelets().size(), lanes.at(5)->getContainedLanelets().size()};
    EXPECT_EQ(expLaneSizes, laneSizes);
    startIds = {
        lanes.at(0)->getContainedLanelets().front()->getId(), lanes.at(1)->getContainedLanelets().front()->getId(),
        lanes.at(2)->getContainedLanelets().front()->getId(), lanes.at(3)->getContainedLanelets().front()->getId(),
        lanes.at(4)->getContainedLanelets().front()->getId(), lanes.at(5)->getContainedLanelets().front()->getId()};
    expStartIds = {77062, 77695};
    EXPECT_EQ(startIds, expStartIds);
    endIds = {lanes.at(0)->getContainedLanelets().back()->getId(), lanes.at(1)->getContainedLanelets().back()->getId(),
              lanes.at(2)->getContainedLanelets().back()->getId(), lanes.at(3)->getContainedLanelets().back()->getId(),
              lanes.at(4)->getContainedLanelets().back()->getId(), lanes.at(5)->getContainedLanelets().back()->getId()};
    expEndIds = {77071, 77236, 77301};
    EXPECT_EQ(endIds, expEndIds);
}