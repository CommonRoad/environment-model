//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>

#include "test_lane.h"

void LaneTestInitialization::setUpLane() {
    setUpLanelets();
    laneOne = lane_operations::createLaneByContainedLanelets({laneletThree, laneletOne, laneletTwo}, 1000);
    laneTwo = lane_operations::createLaneByContainedLanelets({laneletFour}, 1001);
    laneThree = lane_operations::createLaneByContainedLanelets({laneletFive}, 1002);
}

void LaneTest::SetUp() { setUpLane(); }

TEST_F(LaneTest, Initialization) {
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->getId(), laneletThree->getId());
    EXPECT_EQ(laneOne->getContainedLanelets().at(1)->getId(), laneletOne->getId());
    EXPECT_EQ(laneOne->getContainedLanelets().at(2)->getId(), laneletTwo->getId());
    EXPECT_EQ(laneOne->getId(), 1000);
    EXPECT_EQ(laneOne->getCenterVertices().begin()->x, laneletThree->getCenterVertices().begin()->x);
    EXPECT_EQ(laneOne->getCenterVertices().begin()->y, laneletThree->getCenterVertices().begin()->y);
    EXPECT_EQ(laneOne->getCenterVertices().at(17).x, laneletTwo->getCenterVertices().at(6).x);
    EXPECT_EQ(laneOne->getCenterVertices().at(17).y, laneletTwo->getCenterVertices().at(6).y);
}

TEST_F(LaneTest, ConvertPoint) {
    Eigen::Vector2d originalPoint{3, 2};
    Eigen::Vector2d convertedPoint =
        laneOne->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(originalPoint.x(), originalPoint.y());
    Eigen::Vector2d convertedBack =
        laneOne->getCurvilinearCoordinateSystem()->convertToCartesianCoords(convertedPoint.x(), convertedPoint.y());
    EXPECT_NEAR(originalPoint.x(), convertedBack.x(), 0.0005);
    EXPECT_EQ(originalPoint.y(), convertedBack.y());
}

TEST_F(LaneTest, CurvilinearCoordinateSystem1) {
    Eigen::Vector2d originalPoint{367.370000, -26.210000};
    geometry::EigenPolyline reference_path{};
    std::string refInfo;
    for (int i{0}; i < 42; ++i)
        reference_path.push_back({-40 + i * 12, -26.2350});
    geometry::util::resample_polyline(reference_path, 2, reference_path);
    auto ccs{CurvilinearCoordinateSystem(reference_path, 20., 0.1, 0.00000001)};
    Eigen::Vector2d convertedPoint = ccs.convertToCurvilinearCoords(originalPoint.x(), originalPoint.y());
    Eigen::Vector2d convertedBack = ccs.convertToCartesianCoords(convertedPoint.x(), convertedPoint.y());
    EXPECT_NEAR(originalPoint.x(), convertedBack.x(), 0.0005);
    EXPECT_EQ(originalPoint.y(), convertedBack.y());
}

TEST_F(LaneTest, CurvilinearCoordinateSystem2) {
    Eigen::Vector2d originalPoint{3.000000, 5.250000};
    geometry::EigenPolyline reference_path{
        {-0.000300, 5.250000},  {-0.000200, 5.250000},  {-0.000100, 5.250000},  {0.000000, 5.250000},
        {2.000000, 5.250000},   {4.000000, 5.250000},   {6.000000, 5.250000},   {8.000000, 5.250000},
        {10.000000, 5.250000},  {12.000000, 5.250000},  {14.000000, 5.250000},  {16.000000, 5.250000},
        {18.000000, 5.250000},  {20.000000, 5.250000},  {22.000000, 5.250000},  {24.000000, 5.250000},
        {26.000000, 5.250000},  {28.000000, 5.250000},  {30.000000, 5.250000},  {32.000000, 5.250000},
        {34.000000, 5.250000},  {36.000000, 5.250000},  {38.000000, 5.250000},  {40.000000, 5.250000},
        {42.000000, 5.250000},  {44.000000, 5.250000},  {46.000000, 5.250000},  {48.000000, 5.250000},
        {50.000000, 5.250000},  {52.000000, 5.250000},  {54.000000, 5.250000},  {56.000000, 5.250000},
        {58.000000, 5.250000},  {60.000000, 5.250000},  {62.000000, 5.250000},  {64.000000, 5.250000},
        {66.000000, 5.250000},  {68.000000, 5.250000},  {70.000000, 5.250000},  {72.000000, 5.250000},
        {74.000000, 5.250000},  {76.000000, 5.250000},  {78.000000, 5.250000},  {80.000000, 5.250000},
        {82.000000, 5.250000},  {84.000000, 5.250000},  {86.000000, 5.250000},  {88.000000, 5.250000},
        {90.000000, 5.250000},  {92.000000, 5.250000},  {94.000000, 5.250000},  {96.000000, 5.250000},
        {98.000000, 5.250000},  {100.000000, 5.250000}, {102.000000, 5.250000}, {104.000000, 5.250000},
        {106.000000, 5.250000}, {108.000000, 5.250000}, {110.000000, 5.250000}, {112.000000, 5.250000},
        {114.000000, 5.250000}, {116.000000, 5.250000}, {118.000000, 5.250000}, {120.000000, 5.250000},
        {122.000000, 5.250000}, {124.000000, 5.250000}, {126.000000, 5.250000}, {128.000000, 5.250000},
        {130.000000, 5.250000}, {132.000000, 5.250000}, {134.000000, 5.250000}, {136.000000, 5.250000},
        {138.000000, 5.250000}, {140.000000, 5.250000}, {142.000000, 5.250000}, {144.000000, 5.250000},
        {146.000000, 5.250000}, {148.000000, 5.250000}, {150.000000, 5.250000}, {152.000000, 5.250000},
        {154.000000, 5.250000}, {156.000000, 5.250000}, {158.000000, 5.250000}, {160.000000, 5.250000},
        {162.000000, 5.250000}, {164.000000, 5.250000}, {166.000000, 5.250000}, {168.000000, 5.250000},
        {170.000000, 5.250000}, {172.000000, 5.250000}, {174.000000, 5.250000}, {176.000000, 5.250000},
        {178.000000, 5.250000}, {180.000000, 5.250000}, {182.000000, 5.250000}, {184.000000, 5.250000},
        {186.000000, 5.250000}, {188.000000, 5.250000}, {190.000000, 5.250000}, {192.000000, 5.250000},
        {194.000000, 5.250000}, {196.000000, 5.250000}, {198.000000, 5.250000}, {200.000000, 5.250000},
        {200.000100, 5.250000}, {200.000200, 5.250000}};
    geometry::util::resample_polyline(reference_path, 2, reference_path);
    auto ccs{CurvilinearCoordinateSystem(reference_path, 20., 0.1, 0.00000001)};
    Eigen::Vector2d convertedPoint = ccs.convertToCurvilinearCoords(originalPoint.x(), originalPoint.y());
    Eigen::Vector2d convertedBack = ccs.convertToCartesianCoords(convertedPoint.x(), convertedPoint.y());
    EXPECT_NEAR(originalPoint.x(), convertedBack.x(), 0.0005);
    EXPECT_EQ(originalPoint.y(), convertedBack.y());
}

TEST_F(LaneTest, CheckIntersection) {
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(1)->checkIntersection(polygonOne, ContainmentType::PARTIALLY_CONTAINED),
        true);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(1)->checkIntersection(polygonTwo, ContainmentType::PARTIALLY_CONTAINED),
        true);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(1)->checkIntersection(polygonThree, ContainmentType::PARTIALLY_CONTAINED),
        false);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(1)->checkIntersection(polygonOne, ContainmentType::COMPLETELY_CONTAINED),
        true);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(1)->checkIntersection(polygonTwo, ContainmentType::COMPLETELY_CONTAINED),
        false);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(1)->checkIntersection(polygonThree, ContainmentType::COMPLETELY_CONTAINED),
        false);

    EXPECT_EQ(laneOne->checkIntersection(polygonOne, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->checkIntersection(polygonTwo, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->checkIntersection(polygonThree, ContainmentType::PARTIALLY_CONTAINED), false);
    EXPECT_EQ(laneOne->checkIntersection(polygonOne, ContainmentType::COMPLETELY_CONTAINED), true);
    EXPECT_EQ(laneOne->checkIntersection(polygonTwo, ContainmentType::COMPLETELY_CONTAINED), false);
    EXPECT_EQ(laneOne->checkIntersection(polygonThree, ContainmentType::COMPLETELY_CONTAINED), false);
}

TEST_F(LaneTest, GetSuccessorLanelets) {
    EXPECT_EQ(laneOne->getSuccessorLanelets(laneletOne).size(), 1);
    EXPECT_EQ(laneOne->getSuccessorLanelets(laneletOne).at(0)->getId(), 2);
    EXPECT_EQ(laneOne->getSuccessorLanelets(laneletTwo).size(), 0);
    EXPECT_EQ(laneOne->getSuccessorLanelets(laneletThree).size(), 2);
    EXPECT_EQ(laneOne->getSuccessorLanelets(laneletThree).at(0)->getId(), 1);
    EXPECT_EQ(laneOne->getSuccessorLanelets(laneletThree).at(1)->getId(), 2);

    EXPECT_EQ(laneTwo->getSuccessorLanelets(laneletFour).size(), 0);

    EXPECT_EQ(laneThree->getSuccessorLanelets(laneletFive).size(), 0);
    EXPECT_EQ(laneThree->getSuccessorLanelets(laneletOne).size(), 0);
}