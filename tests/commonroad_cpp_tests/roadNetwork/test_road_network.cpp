//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_road_network.h"
#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

void RoadNetworkTestInitialization::setUpRoadNetwork() {
    std::vector<std::shared_ptr<Lanelet>> lanelets{laneletOne,  laneletTwo, laneletThree, laneletFour,
                                                   laneletFive, laneletSix, laneletSeven};

    roadNetwork = std::make_shared<RoadNetwork>(
        RoadNetwork(lanelets, SupportedTrafficSignCountry::GERMANY, {}, {}, {intersection1, intersection2}));
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    roadNetwork->setIdCounterRef(globalIdRef);
}

void RoadNetworkTest::SetUp() {
    setUpLane();
    setUpIncoming();
    setUpIntersection();
    setUpRoadNetwork();
}

TEST_F(RoadNetworkTest, InitializationComplete) {
    EXPECT_EQ(roadNetwork->getLaneletNetwork().size(), 7);
    EXPECT_EQ(roadNetwork->getLaneletNetwork().at(0)->getId(), 1);
}

TEST_F(RoadNetworkTest, FindOccupiedLaneletsByShape) {
    EXPECT_EQ(roadNetwork->findOccupiedLaneletsByShape(polygonOne).size(), 3); // order can be random
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

TEST_F(RoadNetworkTest, AddLanes) {
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/predicates/DEU_TrafficLightTest-1_1_T-1.xml"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFile);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    roadNetworkScenario->setIdCounterRef(globalIdRef);
    auto lanes{lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(10)},
                                                               roadNetworkScenario)};
    auto updatedLanes{roadNetworkScenario->addLanes(lanes, 10)};
    EXPECT_EQ(lanes.size(), 3);
    EXPECT_EQ(lanes.size(), updatedLanes.size());
    EXPECT_EQ(lanes.at(0)->getId(), 123456789 + 1);
    EXPECT_EQ(lanes.at(1)->getId(), 123456789 + 2);
    EXPECT_EQ(lanes.at(2)->getId(), 123456789 + 3);

    lanes =
        lanelet_operations::createLanesBySingleLanelets({roadNetworkScenario->findLaneletById(4)}, roadNetworkScenario);
    Lanelet let{100000, lanes.at(0)->getLeftBorderVertices(), lanes.at(0)->getRightBorderVertices(),
                lanes.at(0)->getLaneletTypes()};
    geometry::EigenPolyline reference_path;
    for (auto vert : let.getCenterVertices())
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));
    geometry::util::resample_polyline(reference_path, 2, reference_path);
    auto newLane{std::make_shared<Lane>(lanes.at(0)->getContainedLanelets(), let,
                                        std::make_shared<CurvilinearCoordinateSystem>(reference_path))};
    std::vector<std::shared_ptr<Lane>> testLanes{newLane};
    updatedLanes = roadNetworkScenario->addLanes(testLanes, 4);
    EXPECT_EQ(lanes.size(), 1);
    EXPECT_EQ(lanes.size(), updatedLanes.size());
    EXPECT_NE(testLanes.at(0)->getId(), updatedLanes.at(0)->getId());
}

TEST_F(RoadNetworkTest, GetIntersections) {
    EXPECT_EQ(roadNetwork->getIntersections().size(), 2);
    EXPECT_EQ(roadNetwork->getIntersections().at(0)->getId(), 1000);
    EXPECT_EQ(roadNetwork->getIntersections().at(1)->getId(), 1001);
}
