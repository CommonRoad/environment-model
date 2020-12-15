//
// Created by sebastian on 07.12.20.
//

#include "test_lane.h"

void LaneTest::SetUp() {
    setUpRoadNetwork();
    geometry::EigenPolyline reference_path;
    for(auto vert : laneletOne->getCenterVertices()){
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));
    }

    laneOne = std::make_shared<Lane>(Lane(std::vector<std::shared_ptr<Lanelet>>{laneletOne},
                   *laneletOne,
                   CurvilinearCoordinateSystem(reference_path)));
}

TEST_F(LaneTest, Initialization){
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->getId(), laneletOne->getId());
    EXPECT_EQ(laneOne->getLanelet().getId(), laneletOne->getId());
    compareVerticesVector(laneOne->getLanelet().getCenterVertices(), laneletOne->getCenterVertices());
}

TEST_F(LaneTest, ConvertPoint){

    Eigen::Vector2d originalPoint{3, 2};
    Eigen::Vector2d convertedPoint =
            laneOne->getCurvilinearCoordinateSystem().convertToCurvilinearCoords(originalPoint.x(), originalPoint.y());
    Eigen::Vector2d convertedBack = laneOne->getCurvilinearCoordinateSystem().convertToCartesianCoords(convertedPoint.x(), convertedPoint.y());
    EXPECT_EQ(originalPoint.x(), convertedBack.x());
    EXPECT_EQ(originalPoint.y(), convertedBack.y());
}

TEST_F(LaneTest, CheckIntersection){
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->checkIntersection(polygonOne, PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->checkIntersection(polygonTwo, PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->checkIntersection(polygonThree, PARTIALLY_CONTAINED), false);
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->checkIntersection(polygonOne, COMPLETELY_CONTAINED), true);
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->checkIntersection(polygonTwo, COMPLETELY_CONTAINED), false);
    EXPECT_EQ(laneOne->getContainedLanelets().at(0)->checkIntersection(polygonThree, COMPLETELY_CONTAINED), false);

    EXPECT_EQ(laneOne->getLanelet().checkIntersection(polygonOne, PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->getLanelet().checkIntersection(polygonTwo, PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneOne->getLanelet().checkIntersection(polygonThree, PARTIALLY_CONTAINED), false);
    EXPECT_EQ(laneOne->getLanelet().checkIntersection(polygonOne, COMPLETELY_CONTAINED), true);
    EXPECT_EQ(laneOne->getLanelet().checkIntersection(polygonTwo, COMPLETELY_CONTAINED), false);
    EXPECT_EQ(laneOne->getLanelet().checkIntersection(polygonThree, COMPLETELY_CONTAINED), false);
}