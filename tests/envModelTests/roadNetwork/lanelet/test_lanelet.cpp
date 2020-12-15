//
// Created by sebastian on 03.12.20.
//

#include "test_lanelet.h"
#include "roadNetwork/regulatory_elements/traffic_light.h"

namespace bg = boost::geometry;
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

void LaneletTest::SetUp() {
    setUpRoadNetwork();
}

TEST_F(LaneletTest, InitializationComplete){
    //middle lanelet
    compareVerticesVector(laneletOne->getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne->getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne->getRightBorderVertices(), rightBorderOne);
    EXPECT_EQ(laneletOne->getId(), idOne);
    RoadNetworkTest::evaluateTypes(laneletOne->getLaneletType(), laneletTypeOne);
    RoadNetworkTest::evaluateTypes(laneletOne->getUserOneWay(), userOneWayOne);
    RoadNetworkTest::evaluateTypes(laneletOne->getUserBidirectional(), userBidirectionalOne);
    EXPECT_EQ(laneletOne->getSuccessors()[0]->getId(), 2);
    EXPECT_EQ(laneletOne->getSuccessors()[1]->getId(), 6);
    EXPECT_EQ(laneletOne->getPredecessors()[0]->getId(), 3);
    EXPECT_EQ(laneletOne->getPredecessors()[1]->getId(), 7);
    EXPECT_EQ(laneletOne->getAdjacentLeft().adj->getId(), 5);
    EXPECT_EQ(laneletOne->getAdjacentRight().adj->getId(), 4);
    EXPECT_EQ(laneletOne->getTrafficSigns()[0]->getId(), 123);
    EXPECT_EQ(laneletOne->getTrafficLights()[0]->getId(), 456);
    EXPECT_EQ(laneletOne->getStopLine().getPoints()[0].x, 1);
}

TEST_F(LaneletTest, InitializationManual){
    //front lanelet
    EXPECT_EQ(laneletTwo->getId(), idTwo);
    RoadNetworkTest::evaluateTypes(laneletTwo->getLaneletType(), laneletTypeTwo);
    RoadNetworkTest::evaluateTypes(laneletTwo->getUserOneWay(), userOneWayTwo);
    RoadNetworkTest::evaluateTypes(laneletTwo->getUserBidirectional(), userBidirectionalTwo);
    compareVerticesVector(laneletTwo->getCenterVertices(), centerVerticesTwo);
    compareVerticesVector(laneletTwo->getLeftBorderVertices(), leftBorderTwo);
    compareVerticesVector(laneletTwo->getRightBorderVertices(), rightBorderTwo);
}

TEST_F(LaneletTest, InitializationEnd){
    //rear lanelet
    EXPECT_EQ(laneletThree->getId(), idThree);
    RoadNetworkTest::evaluateTypes(laneletThree->getLaneletType(), laneletTypeThree);
    compareVerticesVector(laneletThree->getCenterVertices(), centerVerticesThree);
    compareVerticesVector(laneletThree->getLeftBorderVertices(), leftBorderThree);
    compareVerticesVector(laneletThree->getRightBorderVertices(), rightBorderThree);
}

TEST_F(LaneletTest, ApplyIntersectionTesting){
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonOne), true);
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonTwo), true);
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonThree), false);
}

TEST_F(LaneletTest, CheckIntersection){
    EXPECT_EQ(laneletOne->checkIntersection(polygonOne, PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonTwo, PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonThree, PARTIALLY_CONTAINED), false);
    EXPECT_EQ(laneletOne->checkIntersection(polygonOne, COMPLETELY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonTwo, COMPLETELY_CONTAINED), false);
    EXPECT_EQ(laneletOne->checkIntersection(polygonThree, COMPLETELY_CONTAINED), false);
}

TEST_F(LaneletTest, ConstructOuterPolygon){
    //evaluates whether in setUp creates outer polygon is valid
    //lanelet one
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[0].x(), 0.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[0].y(), 1.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[1].x(), 5.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[1].y(), 1.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[2].x(), 5.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[2].y(), 0.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[3].x(), 0.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[3].y(), 0.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[4].x(), 0.0);
    EXPECT_EQ(laneletOne->getOuterPolygon().outer()[4].y(), 1.0);

    //lanelet two
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[0].x(), 6.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[0].y(), 1.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[1].x(), 11.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[1].y(), 1.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[2].x(), 12.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[2].y(), 2.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[3].x(), 12.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[3].y(), 1.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[4].x(), 11.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[4].y(), 0.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[5].x(), 6.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[5].y(), 0.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[6].x(), 6.0);
    EXPECT_EQ(laneletTwo->getOuterPolygon().outer()[6].y(), 1.0);
}

TEST_F(LaneletTest, GetBoundingBox){
    //lanelet one
    EXPECT_EQ(laneletOne->getBoundingBox().max_corner().x(), 5.0);
    EXPECT_EQ(laneletOne->getBoundingBox().max_corner().y(), 1.0);
    EXPECT_EQ(laneletOne->getBoundingBox().min_corner().x(), 0.0);
    EXPECT_EQ(laneletOne->getBoundingBox().min_corner().y(), 0.0);

    //lanelet two
    EXPECT_EQ(laneletTwo->getBoundingBox().max_corner().x(), 12.0);
    EXPECT_EQ(laneletTwo->getBoundingBox().max_corner().y(), 2.0);
    EXPECT_EQ(laneletTwo->getBoundingBox().min_corner().x(), 6.0);
    EXPECT_EQ(laneletTwo->getBoundingBox().min_corner().y(), 0.0);
}

TEST_F(LaneletTest, GetOrientationAtPosition){
    EXPECT_EQ(laneletOne->getOrientationAtPosition(0.0, 0.0), 0.0);
    EXPECT_EQ(laneletOne->getOrientationAtPosition(1.0, 0.0), 0.0);
    EXPECT_EQ(laneletTwo->getOrientationAtPosition(10.0, 0.5), 0.0);
    EXPECT_EQ(laneletTwo->getOrientationAtPosition(11.0, 0.5), 0.78539816339744828);
    EXPECT_EQ(laneletTwo->getOrientationAtPosition(11.5, 0.75), 0.78539816339744828);
    EXPECT_EQ(laneletTwo->getOrientationAtPosition(12.0, 1.5), 0.78539816339744828);
}