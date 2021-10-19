//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

#include "test_lane.h"

void LaneTestInitialization::setUpLane() {
    setUpLanelets();
    geometry::EigenPolyline reference_path;
    for (auto vert : laneletThree->getCenterVertices()) {
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));
    }
    for (auto vert : laneletOne->getCenterVertices()) {
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));
    }
    for (auto vert : laneletTwo->getCenterVertices()) {
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));
    }

    laneOne = std::make_shared<Lane>(Lane(std::vector<std::shared_ptr<Lanelet>>{laneletOne}, *laneletOne,
                                          CurvilinearCoordinateSystem(reference_path)));
}

void LaneTest::SetUp() { setUpLane(); }

TEST_F(LaneTest, Initialization) {
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->getId(), laneletOne->getId());
    EXPECT_EQ(laneOne->getId(), laneletOne->getId());
    compareVerticesVector(laneOne->getCenterVertices(), laneletOne->getCenterVertices());
}

TEST_F(LaneTest, ConvertPoint) {
    Eigen::Vector2d originalPoint{3, 2};
    Eigen::Vector2d convertedPoint =
        laneOne->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(originalPoint.x(), originalPoint.y());
    Eigen::Vector2d convertedBack =
        laneOne->getCurvilinearCoordinateSystem().convertToCartesianCoords(convertedPoint.x(), convertedPoint.y());
    EXPECT_NEAR(originalPoint.x(), convertedBack.x(), 0.0005);
    EXPECT_EQ(originalPoint.y(), convertedBack.y());
}

TEST_F(LaneTest, CurvilinearCoordinateSystem) {
    Eigen::Vector2d originalPoint{367.370000, -26.210000};
    geometry::EigenPolyline reference_path{};
    std::string refInfo;
    for (int i{0}; i < 42; ++i) {
        reference_path.push_back({-40 + i * 12, -26.2350});
        refInfo += "{" + std::to_string(reference_path[i].x()) + ", " + std::to_string(reference_path[i].y()) + "}";
    }
    std::cout << refInfo << '\n';
    geometry::util::resample_polyline(reference_path, 2, reference_path);
    auto ccs{CurvilinearCoordinateSystem(reference_path, 20., 0.1, 0.00000001)};
    refInfo = "";
    for (const auto &ref : ccs.referencePath())
        refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}";
    std::cout << refInfo << '\n';
    Eigen::Vector2d convertedPoint = ccs.convertToCurvilinearCoords(originalPoint.x(), originalPoint.y());
    Eigen::Vector2d convertedBack = ccs.convertToCartesianCoords(convertedPoint.x(), convertedPoint.y());
    EXPECT_NEAR(originalPoint.x(), convertedBack.x(), 0.0005);
    EXPECT_EQ(originalPoint.y(), convertedBack.y());
}

TEST_F(LaneTest, CheckIntersection) {
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(0)->checkIntersection(polygonOne, ContainmentType::PARTIALLY_CONTAINED),
        true);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(0)->checkIntersection(polygonTwo, ContainmentType::PARTIALLY_CONTAINED),
        true);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(0)->checkIntersection(polygonThree, ContainmentType::PARTIALLY_CONTAINED),
        false);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(0)->checkIntersection(polygonOne, ContainmentType::COMPLETELY_CONTAINED),
        true);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(0)->checkIntersection(polygonTwo, ContainmentType::COMPLETELY_CONTAINED),
        false);
    EXPECT_EQ(
        laneOne->getContainedLanelets().at(0)->checkIntersection(polygonThree, ContainmentType::COMPLETELY_CONTAINED),
        false);

    EXPECT_EQ(laneOne->checkIntersection(polygonOne, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->checkIntersection(polygonTwo, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->checkIntersection(polygonThree, ContainmentType::PARTIALLY_CONTAINED), false);
    EXPECT_EQ(laneOne->checkIntersection(polygonOne, ContainmentType::COMPLETELY_CONTAINED), true);
    EXPECT_EQ(laneOne->checkIntersection(polygonTwo, ContainmentType::COMPLETELY_CONTAINED), false);
    EXPECT_EQ(laneOne->checkIntersection(polygonThree, ContainmentType::COMPLETELY_CONTAINED), false);
}