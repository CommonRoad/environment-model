//
// Created by sebastian on 03.12.20.
//

#include "test_lanelet.h"
#include "roadNetwork/regulatory_elements/traffic_light.h"

namespace bg = boost::geometry;
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

void LaneletTestInitialization::compareVerticesVector(std::vector<vertice> verticesOne, std::vector<vertice> verticesTwo){
    for(int i=0; i<verticesOne.size(); ++i){
        EXPECT_EQ(verticesOne.at(i).x, verticesTwo.at(i).x);
        EXPECT_EQ(verticesOne.at(i).y, verticesTwo.at(i).y);
    }
}

void LaneletTestInitialization::setUpLanelets(){
    //middle lanelet
    idLaneletOne = 1;
    laneletTypeLaneletOne = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    userOneWayLaneletOne = std::vector<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    userBidirectionalLaneletOne = std::vector<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    centerVerticesLaneletOne = std::vector<vertice>{vertice{0, 0.5}, vertice{1, 0.5}, vertice{2, 0.5},
                                                    vertice{3, 0.5}, vertice{4, 0.5}, vertice{5, 0.5}};
    leftBorderLaneletOne = std::vector<vertice>{vertice{0, 1}, vertice{1, 1}, vertice{2, 1},
                                                vertice{3, 1.0}, vertice{4, 1.0}, vertice{5, 1.0}};
    rightBorderLaneletOne = std::vector<vertice>{vertice{0, 0}, vertice{1, 0}, vertice{2, 0},
                                                 vertice{3, 0}, vertice{4, 0}, vertice{5, 0}};
    laneletOne = std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                                   laneletTypeLaneletOne, userOneWayLaneletOne,
                                                   userBidirectionalLaneletOne));

    //front lanelet
    laneletTwo = std::make_shared<Lanelet>(Lanelet());
    idLaneletTwo = 2;
    laneletTypeLaneletTwo = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    userOneWayLaneletTwo = std::vector<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    userBidirectionalLaneletTwo = std::vector<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    centerVerticesLaneletTwo = std::vector<vertice>{vertice{6, 0.5}, vertice{7, 0.5}, vertice{8, 0.5},
                                                    vertice{9, 0.5}, vertice{10, 0.5}, vertice{11, 0.5}};
    leftBorderLaneletTwo = std::vector<vertice>{vertice{6, 1.0}, vertice{7, 1.0}, vertice{8, 1.0},
                                                vertice{9, 1.0}, vertice{10, 1.0}, vertice{11, 1.0}};
    rightBorderLaneletTwo = std::vector<vertice>{vertice{6, 0}, vertice{7, 0}, vertice{8, 0},
                                                 vertice{9, 0.0}, vertice{10, 0.0}, vertice{11, 0.0}};
    laneletTwo->setId(idLaneletTwo);
    laneletTwo->setLaneletType(laneletTypeLaneletTwo);
    laneletTwo->setRightBorderVertices(rightBorderLaneletTwo);
    laneletTwo->setLeftBorderVertices(leftBorderLaneletTwo);
    laneletTwo->setUserBidirectional(userBidirectionalLaneletTwo);
    laneletTwo->setUserOneWay(userOneWayLaneletTwo);
    laneletTwo->createCenterVertices();
    laneletTwo->constructOuterPolygon();
    laneletTwo->addLeftVertex(vertice{12, 2.0});
    laneletTwo->addRightVertex(vertice{12, 1.0});
    laneletTwo->addCenterVertex(vertice{12, 1.5});
    laneletTwo->constructOuterPolygon();
    leftBorderLaneletTwo.push_back(vertice{12, 2.0});
    rightBorderLaneletTwo.push_back(vertice{12, 1.0});
    centerVerticesLaneletTwo.push_back(vertice{12, 1.5});

    //rear lanelet
    idLaneletThree = 3;
    laneletTypeLaneletThree = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    centerVerticesLaneletThree = std::vector<vertice>{vertice{-5, 0.5}, vertice{-4, 0.5}, vertice{-3, 0.5},
                                                      vertice{-2, 0.5}, vertice{-1, 0.5}, vertice{0, 0.5}};
    leftBorderLaneletThree = std::vector<vertice>{vertice{-5, 1}, vertice{-4, 1}, vertice{-3, 1},
                                                  vertice{-2, 1.0}, vertice{-1, 1.0}, vertice{0, 1.0}};
    rightBorderLaneletThree = std::vector<vertice>{vertice{-5, 0}, vertice{-4, 0}, vertice{-3, 0},
                                                   vertice{-2, 0.0}, vertice{-1, 0.0}, vertice{0, 0.0}};
    Lanelet laneletThreeTmp = Lanelet();
    laneletThreeTmp.setId(idLaneletThree);
    laneletThreeTmp.setLaneletType(laneletTypeLaneletThree);
    laneletThreeTmp.setLeftBorderVertices(leftBorderLaneletThree);
    laneletThreeTmp.setRightBorderVertices(rightBorderLaneletThree);
    laneletThreeTmp.createCenterVertices();
    laneletThreeTmp.constructOuterPolygon();
    laneletThree = std::make_shared<Lanelet>(laneletThreeTmp);

    //right lanelet
    int idFour{ 4 };
    std::vector<LaneletType> laneletTypeFour{ std::vector<LaneletType>{LaneletType::urban} };
    std::vector<vertice> leftBorderFour{vertice{6, 0.0}, vertice{7, 0.0}, vertice{8, 0.0},
                                        vertice{9, 0.0}, vertice{10, 0.0}, vertice{11, 0.0}};
    std::vector<vertice> rightBorderFour{vertice{6, -1.0}, vertice{7, -1.0}, vertice{8, -1.0},
                                         vertice{9, -1.0}, vertice{10, -1.0}, vertice{11, -1.0}};
    laneletFour = std::make_shared<Lanelet>(Lanelet(idFour, leftBorderFour, rightBorderFour, laneletTypeFour));

    //left lanelet
    int idFive{ 5 };
    std::vector<LaneletType> laneletTypeFive{ std::vector<LaneletType>{LaneletType::highway, LaneletType::crosswalk} };
    std::vector<vertice> leftBorderFive{vertice{6, 2.0}, vertice{7, 2.0}, vertice{8, 2.0},
                                        vertice{9, 2.0}, vertice{10, 2.0}, vertice{11, 2.0}};
    std::vector<vertice> rightBorderFive{vertice{6, 1.0}, vertice{7, 1.0}, vertice{8, 1.0},
                                         vertice{9, 1.0}, vertice{10, 1.0}, vertice{11, 1.0}};
    laneletFive = std::make_shared<Lanelet>(Lanelet(idFive, leftBorderFive, rightBorderFive, laneletTypeFive));

    //second front lanelet
    int idSix{ 6 };
    std::vector<LaneletType> laneletTypeSix{ std::vector<LaneletType>{LaneletType::busLane,
                                                                      LaneletType::urban} };
    std::vector<vertice> leftBorderSix = std::vector<vertice>{vertice{6, 1.0}, vertice{7, 1.0},
                                                              vertice{8, 1.0}};
    std::vector<vertice> rightBorderSix = std::vector<vertice>{vertice{6, 0}, vertice{7, 0},
                                                               vertice{8, 0}};
    laneletSix = std::make_shared<Lanelet>(Lanelet(idSix, leftBorderSix, rightBorderSix, laneletTypeSix));

    //second rear lanelet
    int idSeven{ 7 };
    std::vector<LaneletType> laneletTypeSeven{ std::vector<LaneletType>{LaneletType::urban, LaneletType::country} };
    std::vector<vertice> leftBorderSeven{vertice{-2, 1.5}, vertice{-1, 2}, vertice{0, 2}};
    std::vector<vertice> rightBorderSeven{vertice{-2, 0.0}, vertice{-1, 0.0}, vertice{0, 0.0}};
    laneletSeven = std::make_shared<Lanelet>(Lanelet(idSeven, leftBorderSeven, rightBorderSeven, laneletTypeSeven));

    //traffic sign, traffic light, and stop line for middle lanelet
    TrafficSignElement tsElem = TrafficSignElement("123");
    std::shared_ptr<TrafficSign> ts = std::make_shared<TrafficSign>(TrafficSign());
    ts->addTrafficSignElement(tsElem);
    ts->setId(123);
    std::shared_ptr<TrafficLight> tl = std::make_shared<TrafficLight>(TrafficLight());
    tl->setId(456);
    tl->setActive(true);
    tl->setOffset(0.0);
    tl->setDirection(TrafficLightDirection::straight);
    tl->setCycle(std::vector<CycleElement>{CycleElement{CycleElementType::green, 1},
                                           CycleElement{CycleElementType::yellow, 1},
                                           CycleElement{CycleElementType::red, 1},
                                           CycleElement{CycleElementType::red_yellow, 1}});

    StopLine sl = StopLine();
    sl.setLineMarking(LineMarking::broad_solid);
    sl.setPoints(std::vector<vertice>{vertice{1, 2}, vertice{3, 4}});
    sl.setTrafficSign(ts);
    sl.setTrafficLight(tl);

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
    laneletOne->addTrafficLight(tl);
    laneletOne->addTrafficSign(ts);
    laneletOne->setStopLine(sl);

    polygonOne = polygon_type{{{0.0, 0.0}, {0.0, 0.5}, {0.5, 0.5}, {0.5, 0.0},
                                      {0.0, 0.0}}};
    polygonTwo = polygon_type{{{0.5, 0.5}, {0.5, 2.0}, {1.0, 2.0}, {1.0, 0.5},
                                      {0.5, 0.5}}};
    polygonThree = polygon_type{{{10.0, 10.0}, {10.0, 12.0}, {11.0, 12.0}, {11.0, 10.0},
                                        {10.0, 10.0}}};


}

void LaneletTest::SetUp() {
    setUpLanelets();
}

TEST_F(LaneletTest, InitializationComplete){
    //middle lanelet
    compareVerticesVector(laneletOne->getCenterVertices(), centerVerticesLaneletOne);
    compareVerticesVector(laneletOne->getLeftBorderVertices(), leftBorderLaneletOne);
    compareVerticesVector(laneletOne->getRightBorderVertices(), rightBorderLaneletOne);
    EXPECT_EQ(laneletOne->getId(), idLaneletOne);
    evaluateTypes(laneletOne->getLaneletType(), laneletTypeLaneletOne);
    evaluateTypes(laneletOne->getUserOneWay(), userOneWayLaneletOne);
    evaluateTypes(laneletOne->getUserBidirectional(), userBidirectionalLaneletOne);
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
    EXPECT_EQ(laneletTwo->getId(), idLaneletTwo);
    evaluateTypes(laneletTwo->getLaneletType(), laneletTypeLaneletTwo);
    evaluateTypes(laneletTwo->getUserOneWay(), userOneWayLaneletTwo);
    evaluateTypes(laneletTwo->getUserBidirectional(), userBidirectionalLaneletTwo);
    compareVerticesVector(laneletTwo->getCenterVertices(), centerVerticesLaneletTwo);
    compareVerticesVector(laneletTwo->getLeftBorderVertices(), leftBorderLaneletTwo);
    compareVerticesVector(laneletTwo->getRightBorderVertices(), rightBorderLaneletTwo);
}

TEST_F(LaneletTest, InitializationEnd){
    //rear lanelet
    EXPECT_EQ(laneletThree->getId(), idLaneletThree);
    evaluateTypes(laneletThree->getLaneletType(), laneletTypeLaneletThree);
    compareVerticesVector(laneletThree->getCenterVertices(), centerVerticesLaneletThree);
    compareVerticesVector(laneletThree->getLeftBorderVertices(), leftBorderLaneletThree);
    compareVerticesVector(laneletThree->getRightBorderVertices(), rightBorderLaneletThree);
}

TEST_F(LaneletTest, ApplyIntersectionTesting){
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonOne), true);
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonTwo), true);
    EXPECT_EQ(laneletOne->applyIntersectionTesting(polygonThree), false);
}

TEST_F(LaneletTest, CheckIntersection){
    EXPECT_EQ(laneletOne->checkIntersection(polygonOne, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonTwo, ContainmentType::PARTIALLY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonThree, ContainmentType::PARTIALLY_CONTAINED), false);
    EXPECT_EQ(laneletOne->checkIntersection(polygonOne, ContainmentType::COMPLETELY_CONTAINED), true);
    EXPECT_EQ(laneletOne->checkIntersection(polygonTwo, ContainmentType::COMPLETELY_CONTAINED), false);
    EXPECT_EQ(laneletOne->checkIntersection(polygonThree, ContainmentType::COMPLETELY_CONTAINED), false);
}

TEST_F(LaneletTest, ConstructOuterPolygon){
    //evaluates whether in setUp creates outer polygon is valid (this test case does not check the vertices directly)
    //lanelet one
    std::vector<double> expXVerticesLaneletOne {0.0, 0.0, 5.0, 5.0};
    std::vector<double> expYVerticesLaneletOne {0.0, 0.0, 1.0, 1.0};
    std::vector<double> expXVerticesLaneletTwo {6.0,  6.0, 11.0, 11.0, 12.0, 12.0};
    std::vector<double> expYVerticesLaneletTwo {0.0, 0.0, 1.0, 1.0, 1.0, 2.0};
    std::vector<double> xVerticesResultLaneletOne;
    std::vector<double> yVerticesResultLaneletOne;
    std::vector<double> xVerticesResultLaneletTwo;
    std::vector<double> yVerticesResultLaneletTwo;

    for(int i = 0; i < expXVerticesLaneletOne.size(); ++i){
        xVerticesResultLaneletOne.push_back(laneletOne->getOuterPolygon().outer()[i].x());
        yVerticesResultLaneletOne.push_back(laneletOne->getOuterPolygon().outer()[i].y());
    }
    std::sort(xVerticesResultLaneletOne.begin(), xVerticesResultLaneletOne.end());
    std::sort(yVerticesResultLaneletOne.begin(), yVerticesResultLaneletOne.end());
    for(int i = 0; i < expXVerticesLaneletOne.size(); ++i){
        EXPECT_EQ(xVerticesResultLaneletOne.at(i), expXVerticesLaneletOne.at(i));
        EXPECT_EQ(yVerticesResultLaneletOne.at(i), expYVerticesLaneletOne.at(i));
    }

    for(int i = 0; i < expXVerticesLaneletTwo.size(); ++i){
        xVerticesResultLaneletTwo.push_back(laneletTwo->getOuterPolygon().outer()[i].x());
        yVerticesResultLaneletTwo.push_back(laneletTwo->getOuterPolygon().outer()[i].y());
    }
    std::sort(xVerticesResultLaneletTwo.begin(), xVerticesResultLaneletTwo.end());
    std::sort(yVerticesResultLaneletTwo.begin(), yVerticesResultLaneletTwo.end());
    for(int i = 0; i < expXVerticesLaneletTwo.size(); ++i){
        EXPECT_EQ(xVerticesResultLaneletTwo.at(i), expXVerticesLaneletTwo.at(i));
        EXPECT_EQ(yVerticesResultLaneletTwo.at(i), expYVerticesLaneletTwo.at(i));
    }
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