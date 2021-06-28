//
// Created by sebastian on 07.12.20.
//

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