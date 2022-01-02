//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_lanelet.h"

namespace bg = boost::geometry;
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

void LaneletTestInitialization::compareVerticesVector(std::vector<vertex> verticesOne,
                                                      std::vector<vertex> verticesTwo) {
    for (size_t i = 0; i < verticesOne.size(); ++i) {
        EXPECT_EQ(verticesOne.at(i).x, verticesTwo.at(i).x);
        EXPECT_EQ(verticesOne.at(i).y, verticesTwo.at(i).y);
    }
}

void LaneletTestInitialization::setUpLanelets() {
    // middle lanelet
    idLaneletOne = 1;
    laneletTypeLaneletOne = std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    userOneWayLaneletOne = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    userBidirectionalLaneletOne = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    centerVerticesLaneletOne = std::vector<vertex>{vertex{0, 1.5},  vertex{10, 1.5}, vertex{20, 1.5},
                                                   vertex{30, 1.5}, vertex{40, 1.5}, vertex{50, 1.5}};
    leftBorderLaneletOne = std::vector<vertex>{vertex{0, 3},    vertex{10, 3},   vertex{20, 3},
                                               vertex{30, 3.0}, vertex{40, 3.0}, vertex{50, 3.0}};
    rightBorderLaneletOne =
        std::vector<vertex>{vertex{0, 0}, vertex{10, 0}, vertex{20, 0}, vertex{30, 0}, vertex{40, 0}, vertex{50, 0}};
    laneletOne =
        std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                          laneletTypeLaneletOne, userOneWayLaneletOne, userBidirectionalLaneletOne));

    // front lanelet
    laneletTwo = std::make_shared<Lanelet>(Lanelet());
    idLaneletTwo = 2;
    laneletTypeLaneletTwo = std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    userOneWayLaneletTwo = std::set<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    userBidirectionalLaneletTwo = std::set<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    centerVerticesLaneletTwo = std::vector<vertex>{vertex{60, 1.5}, vertex{70, 1.5},  vertex{80, 1.5},
                                                   vertex{90, 1.5}, vertex{100, 1.5}, vertex{110, 1.5}};
    leftBorderLaneletTwo = std::vector<vertex>{vertex{60, 3.0}, vertex{70, 3.0},  vertex{80, 3.0},
                                               vertex{90, 3.0}, vertex{100, 3.0}, vertex{110, 3.0}};
    rightBorderLaneletTwo = std::vector<vertex>{vertex{60, 0},   vertex{70, 0},    vertex{80, 0},
                                                vertex{90, 0.0}, vertex{100, 0.0}, vertex{110, 0.0}};
    laneletTwo->setId(idLaneletTwo);
    laneletTwo->setLaneletTypes(laneletTypeLaneletTwo);
    laneletTwo->setRightBorderVertices(rightBorderLaneletTwo);
    laneletTwo->setLeftBorderVertices(leftBorderLaneletTwo);
    laneletTwo->setUsersBidirectional(userBidirectionalLaneletTwo);
    laneletTwo->setUsersOneWay(userOneWayLaneletTwo);
    laneletTwo->createCenterVertices();
    laneletTwo->constructOuterPolygon();
    laneletTwo->addLeftVertex(vertex{120, 6.0});
    laneletTwo->addRightVertex(vertex{120, 3.0});
    laneletTwo->addCenterVertex(vertex{120, 4.5});
    laneletTwo->constructOuterPolygon();
    leftBorderLaneletTwo.push_back(vertex{120, 6.0});
    rightBorderLaneletTwo.push_back(vertex{120, 3.0});
    centerVerticesLaneletTwo.push_back(vertex{120, 4.5});

    // rear lanelet
    idLaneletThree = 3;
    laneletTypeLaneletThree = std::set<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    centerVerticesLaneletThree = std::vector<vertex>{vertex{-50, 1.5}, vertex{-40, 1.5}, vertex{-30, 1.5},
                                                     vertex{-20, 1.5}, vertex{-10, 1.5}, vertex{0, 1.5}};
    leftBorderLaneletThree = std::vector<vertex>{vertex{-50, 3},   vertex{-40, 3},   vertex{-30, 3},
                                                 vertex{-20, 3.0}, vertex{-10, 3.0}, vertex{0, 3.0}};
    rightBorderLaneletThree = std::vector<vertex>{vertex{-50, 0},   vertex{-40, 0},   vertex{-30, 0},
                                                  vertex{-20, 0.0}, vertex{-10, 0.0}, vertex{0, 0.0}};
    Lanelet laneletThreeTmp = Lanelet();
    laneletThreeTmp.setId(idLaneletThree);
    laneletThreeTmp.setLaneletTypes(laneletTypeLaneletThree);
    laneletThreeTmp.setLeftBorderVertices(leftBorderLaneletThree);
    laneletThreeTmp.setRightBorderVertices(rightBorderLaneletThree);
    laneletThreeTmp.createCenterVertices();
    laneletThreeTmp.constructOuterPolygon();
    laneletThree = std::make_shared<Lanelet>(laneletThreeTmp);

    // right lanelet
    size_t idFour{4};
    std::set<LaneletType> laneletTypeFour{LaneletType::urban};
    std::vector<vertex> leftBorderFour{vertex{60, 0.0}, vertex{70, 0.0},  vertex{80, 0.0},
                                       vertex{90, 0.0}, vertex{100, 0.0}, vertex{110, 0.0}};
    std::vector<vertex> rightBorderFour{vertex{60, -3.0}, vertex{70, -3.0},  vertex{80, -3.0},
                                        vertex{90, -3.0}, vertex{100, -3.0}, vertex{110, -3.0}};
    laneletFour = std::make_shared<Lanelet>(Lanelet(idFour, leftBorderFour, rightBorderFour, laneletTypeFour));

    // left lanelet
    size_t idFive{5};
    std::set<LaneletType> laneletTypeFive{LaneletType::highway, LaneletType::crosswalk};
    std::vector<vertex> leftBorderFive{vertex{60, 6.0}, vertex{70, 6.0},  vertex{80, 6.0},
                                       vertex{90, 6.0}, vertex{100, 6.0}, vertex{110, 6.0}};
    std::vector<vertex> rightBorderFive{vertex{60, 3.0}, vertex{70, 3.0},  vertex{80, 3.0},
                                        vertex{90, 3.0}, vertex{100, 3.0}, vertex{110, 3.0}};
    laneletFive = std::make_shared<Lanelet>(Lanelet(idFive, leftBorderFive, rightBorderFive, laneletTypeFive));

    // second front lanelet
    size_t idSix{6};
    std::set<LaneletType> laneletTypeSix{LaneletType::busLane, LaneletType::urban};
    std::vector<vertex> leftBorderSix = std::vector<vertex>{vertex{60, 3.0}, vertex{70, 3.0}, vertex{100, 3.0}};
    std::vector<vertex> rightBorderSix = std::vector<vertex>{vertex{60, 0}, vertex{70, 0}, vertex{100, 0}};
    laneletSix = std::make_shared<Lanelet>(Lanelet(idSix, leftBorderSix, rightBorderSix, laneletTypeSix));

    // second rear lanelet
    size_t idSeven{7};
    std::set<LaneletType> laneletTypeSeven{LaneletType::urban, LaneletType::country};
    std::vector<vertex> leftBorderSeven{vertex{-40, 3.5}, vertex{-10, 5}, vertex{0, 5}};
    std::vector<vertex> rightBorderSeven{vertex{-40, 0.0}, vertex{-10, 0.0}, vertex{0, 0.0}};
    laneletSeven = std::make_shared<Lanelet>(Lanelet(idSeven, leftBorderSeven, rightBorderSeven, laneletTypeSeven));

    // traffic sign, traffic light, and stop line for middle lanelet
    std::shared_ptr<TrafficSignElement> tsElem = std::make_shared<TrafficSignElement>("123");
    std::shared_ptr<TrafficSign> sign = std::make_shared<TrafficSign>(TrafficSign());
    sign->addTrafficSignElement(tsElem);
    sign->setId(123);
    std::shared_ptr<TrafficLight> light = std::make_shared<TrafficLight>(TrafficLight());
    light->setId(456);
    light->setActive(true);
    light->setOffset(0.0);
    light->setDirection(TurningDirections::straight);
    light->setCycle(std::vector<TrafficLightCycleElement>{TrafficLightCycleElement{TrafficLightState::green, 1},
                                                          TrafficLightCycleElement{TrafficLightState::yellow, 1},
                                                          TrafficLightCycleElement{TrafficLightState::red, 1},
                                                          TrafficLightCycleElement{TrafficLightState::red_yellow, 1}});

    StopLine sline = StopLine();
    sline.setLineMarking(LineMarking::broad_solid);
    sline.setPoints(std::vector<vertex>{vertex{1, 2}, vertex{3, 4}});
    sline.addTrafficSign(sign);
    sline.addTrafficLight(light);

    // add successors, predecessors, adjacent, traffic sign, traffic light, and stop line to lanelet one
    laneletOne->addSuccessor(laneletTwo);
    laneletTwo->addPredecessor(laneletOne);
    laneletOne->addSuccessor(laneletSix);
    laneletSix->addPredecessor(laneletOne);
    laneletOne->addPredecessor(laneletThree);
    laneletThree->addSuccessor(laneletOne);
    laneletOne->addPredecessor(laneletSeven);
    laneletSeven->addSuccessor(laneletOne);
    laneletOne->setLeftAdjacent(laneletFive, DrivingDirection::opposite);
    laneletFive->setRightAdjacent(laneletOne, DrivingDirection::opposite);
    laneletOne->setRightAdjacent(laneletFour, DrivingDirection::same);
    laneletFour->setLeftAdjacent(laneletOne, DrivingDirection::same);
    laneletOne->addTrafficLight(light);
    laneletOne->addTrafficSign(sign);
    laneletOne->setStopLine(std::make_shared<StopLine>(sline));

    polygonOne = polygon_type{{{0.0, 0.0}, {0.0, 0.5}, {0.5, 0.5}, {0.5, 0.0}, {0.0, 0.0}}};
    polygonTwo = polygon_type{{{0.5, 0.5}, {0.5, 4.0}, {1.0, 4.0}, {1.0, 0.5}, {0.5, 0.5}}};
    polygonThree = polygon_type{{{10.0, 10.0}, {10.0, 12.0}, {11.0, 12.0}, {11.0, 10.0}, {10.0, 10.0}}};
}

void LaneletTest::SetUp() { setUpLanelets(); }

TEST_F(LaneletTest, InitializationComplete) {
    // middle lanelet
    compareVerticesVector(laneletOne->getCenterVertices(), centerVerticesLaneletOne);
    compareVerticesVector(laneletOne->getLeftBorderVertices(), leftBorderLaneletOne);
    compareVerticesVector(laneletOne->getRightBorderVertices(), rightBorderLaneletOne);
    EXPECT_EQ(laneletOne->getId(), idLaneletOne);
    EXPECT_EQ(laneletOne->getLaneletTypes(), laneletTypeLaneletOne);
    EXPECT_EQ(laneletOne->getUsersOneWay(), userOneWayLaneletOne);
    EXPECT_EQ(laneletOne->getUsersBidirectional(), userBidirectionalLaneletOne);
    EXPECT_EQ(laneletOne->getSuccessors()[0]->getId(), 2);
    EXPECT_EQ(laneletOne->getSuccessors()[1]->getId(), 6);
    EXPECT_EQ(laneletOne->getPredecessors()[0]->getId(), 3);
    EXPECT_EQ(laneletOne->getPredecessors()[1]->getId(), 7);
    EXPECT_EQ(laneletOne->getAdjacentLeft().adj->getId(), 5);
    EXPECT_EQ(laneletOne->getAdjacentRight().adj->getId(), 4);
    EXPECT_EQ(laneletOne->getTrafficSigns()[0]->getId(), 123);
    EXPECT_EQ(laneletOne->getTrafficLights()[0]->getId(), 456);
    EXPECT_EQ(laneletOne->getStopLine()->getPoints()[0].x, 1);
}

TEST_F(LaneletTest, InitializationManual) {
    // front lanelet
    EXPECT_EQ(laneletTwo->getId(), idLaneletTwo);
    EXPECT_EQ(laneletTwo->getLaneletTypes(), laneletTypeLaneletTwo);
    EXPECT_EQ(laneletTwo->getUsersOneWay(), userOneWayLaneletTwo);
    EXPECT_EQ(laneletTwo->getUsersBidirectional(), userBidirectionalLaneletTwo);
    compareVerticesVector(laneletTwo->getCenterVertices(), centerVerticesLaneletTwo);
    compareVerticesVector(laneletTwo->getLeftBorderVertices(), leftBorderLaneletTwo);
    compareVerticesVector(laneletTwo->getRightBorderVertices(), rightBorderLaneletTwo);
}

TEST_F(LaneletTest, InitializationEnd) {
    // rear lanelet
    EXPECT_EQ(laneletThree->getId(), idLaneletThree);
    EXPECT_EQ(laneletThree->getLaneletTypes(), laneletTypeLaneletThree);
    compareVerticesVector(laneletThree->getCenterVertices(), centerVerticesLaneletThree);
    compareVerticesVector(laneletThree->getLeftBorderVertices(), leftBorderLaneletThree);
    compareVerticesVector(laneletThree->getRightBorderVertices(), rightBorderLaneletThree);
}

TEST_F(LaneletTest, ApplyIntersectionTesting) {
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonOne), true);
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonTwo), true);
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonThree), false);
}

TEST_F(LaneletTest, CheckIntersection) {
    EXPECT_EQ(laneletOne->checkIntersection(polygonOne, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonTwo, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonThree, ContainmentType::PARTIALLY_CONTAINED), false);
    EXPECT_EQ(laneletOne->checkIntersection(polygonOne, ContainmentType::COMPLETELY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonTwo, ContainmentType::COMPLETELY_CONTAINED), false);
    EXPECT_EQ(laneletOne->checkIntersection(polygonThree, ContainmentType::COMPLETELY_CONTAINED), false);
}

TEST_F(LaneletTest, ConstructOuterPolygon) {
    // evaluates whether in setUp creates outer polygon is valid (this test case does not check the vertices directly)
    // lanelet one
    std::vector<double> expXVerticesLaneletOne{0.0, 0.0, 50.0, 50.0};
    std::vector<double> expYVerticesLaneletOne{0.0, 0.0, 3.0, 3.0};
    std::vector<double> expXVerticesLaneletTwo{60.0, 60.0, 110.0, 110.0, 120.0, 120.0};
    std::vector<double> expYVerticesLaneletTwo{0.0, 0.0, 3.0, 3.0, 3.0, 6.0};
    std::vector<double> xVerticesResultLaneletOne;
    std::vector<double> yVerticesResultLaneletOne;
    std::vector<double> xVerticesResultLaneletTwo;
    std::vector<double> yVerticesResultLaneletTwo;

    for (size_t i = 0; i < expXVerticesLaneletOne.size(); ++i) {
        xVerticesResultLaneletOne.push_back(laneletOne->getOuterPolygon().outer()[i].x());
        yVerticesResultLaneletOne.push_back(laneletOne->getOuterPolygon().outer()[i].y());
    }
    std::sort(xVerticesResultLaneletOne.begin(), xVerticesResultLaneletOne.end());
    std::sort(yVerticesResultLaneletOne.begin(), yVerticesResultLaneletOne.end());
    for (size_t i = 0; i < expXVerticesLaneletOne.size(); ++i) {
        EXPECT_EQ(xVerticesResultLaneletOne.at(i), expXVerticesLaneletOne.at(i));
        EXPECT_EQ(yVerticesResultLaneletOne.at(i), expYVerticesLaneletOne.at(i));
    }

    for (size_t i = 0; i < expXVerticesLaneletTwo.size(); ++i) {
        xVerticesResultLaneletTwo.push_back(laneletTwo->getOuterPolygon().outer()[i].x());
        yVerticesResultLaneletTwo.push_back(laneletTwo->getOuterPolygon().outer()[i].y());
    }
    std::sort(xVerticesResultLaneletTwo.begin(), xVerticesResultLaneletTwo.end());
    std::sort(yVerticesResultLaneletTwo.begin(), yVerticesResultLaneletTwo.end());
    for (size_t i = 0; i < expXVerticesLaneletTwo.size(); ++i) {
        EXPECT_EQ(xVerticesResultLaneletTwo.at(i), expXVerticesLaneletTwo.at(i));
        EXPECT_EQ(yVerticesResultLaneletTwo.at(i), expYVerticesLaneletTwo.at(i));
    }
}

TEST_F(LaneletTest, GetBoundingBox) {
    // lanelet one
    EXPECT_EQ(laneletOne->getBoundingBox().max_corner().x(), 50.0);
    EXPECT_EQ(laneletOne->getBoundingBox().max_corner().y(), 3.0);
    EXPECT_EQ(laneletOne->getBoundingBox().min_corner().x(), 0.0);
    EXPECT_EQ(laneletOne->getBoundingBox().min_corner().y(), 0.0);

    // lanelet two
    EXPECT_EQ(laneletTwo->getBoundingBox().max_corner().x(), 120.0);
    EXPECT_EQ(laneletTwo->getBoundingBox().max_corner().y(), 6.0);
    EXPECT_EQ(laneletTwo->getBoundingBox().min_corner().x(), 60.0);
    EXPECT_EQ(laneletTwo->getBoundingBox().min_corner().y(), 0.0);
}

TEST_F(LaneletTest, GetOrientationAtPosition) {
    EXPECT_EQ(laneletOne->getOrientationAtPosition(0.0, 0.0), 0.0);
    EXPECT_EQ(laneletOne->getOrientationAtPosition(10.0, 0.0), 0.0);
    EXPECT_EQ(laneletTwo->getOrientationAtPosition(100.0, 1.5), 0.0);
    EXPECT_NEAR(laneletTwo->getOrientationAtPosition(110.0, 1.5), 0.291457, 0.00001);
    EXPECT_NEAR(laneletTwo->getOrientationAtPosition(110.5, 2.75), 0.291457, 0.00001);
    EXPECT_NEAR(laneletTwo->getOrientationAtPosition(120.0, 3.5), 0.291457, 0.00001);
}