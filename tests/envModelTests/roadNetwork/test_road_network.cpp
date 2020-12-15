//
// Created by sebastian on 08.12.20.
//

#include "test_road_network.h"

void RoadNetworkTest::compareVerticesVector(std::vector<vertice> verticesOne, std::vector<vertice> verticesTwo){
    for(int i=0; i<verticesOne.size(); ++i){
        EXPECT_EQ(verticesOne.at(i).x, verticesTwo.at(i).x);
        EXPECT_EQ(verticesOne.at(i).y, verticesTwo.at(i).y);
    }
}

void RoadNetworkTest::setUpRoadNetwork(){
    //middle lanelet
    idOne = 1;
    laneletTypeOne = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    userOneWayOne = std::vector<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    userBidirectionalOne = std::vector<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    centerVerticesOne = std::vector<vertice>{vertice{0, 0.5}, vertice{1, 0.5}, vertice{2, 0.5},
                                             vertice{3, 0.5}, vertice{4, 0.5}, vertice{5, 0.5}};
    leftBorderOne = std::vector<vertice>{vertice{0, 1}, vertice{1, 1}, vertice{2, 1},
                                         vertice{3, 1.0}, vertice{4, 1.0}, vertice{5, 1.0}};
    rightBorderOne = std::vector<vertice>{vertice{0, 0}, vertice{1, 0}, vertice{2, 0},
                                          vertice{3, 0}, vertice{4, 0}, vertice{5, 0}};
    laneletOne = std::make_shared<Lanelet>(Lanelet(idOne, leftBorderOne, rightBorderOne,
                                                   laneletTypeOne, userOneWayOne,
                                                   userBidirectionalOne));

    //front lanelet
    laneletTwo = std::make_shared<Lanelet>(Lanelet());
    idTwo = 2;
    laneletTypeTwo = std::vector<LaneletType>{LaneletType::interstate};
    userOneWayTwo = std::vector<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    userBidirectionalTwo = std::vector<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    centerVerticesTwo = std::vector<vertice>{vertice{6, 0.5}, vertice{7, 0.5}, vertice{8, 0.5},
                                             vertice{9, 0.5}, vertice{10, 0.5}, vertice{11, 0.5}};
    leftBorderTwo = std::vector<vertice>{vertice{6, 1.0}, vertice{7, 1.0}, vertice{8, 1.0},
                                         vertice{9, 1.0}, vertice{10, 1.0}, vertice{11, 1.0}};
    rightBorderTwo = std::vector<vertice>{vertice{6, 0}, vertice{7, 0}, vertice{8, 0},
                                          vertice{9, 0.0}, vertice{10, 0.0}, vertice{11, 0.0}};
    laneletTwo->setId(idTwo);
    laneletTwo->setLaneletType(laneletTypeTwo);
    laneletTwo->setRightBorderVertices(rightBorderTwo);
    laneletTwo->setLeftBorderVertices(leftBorderTwo);
    laneletTwo->setUserBidirectional(userBidirectionalTwo);
    laneletTwo->setUserOneWay(userOneWayTwo);
    laneletTwo->createCenterVertices();
    laneletTwo->constructOuterPolygon();
    laneletTwo->addLeftVertex(vertice{12, 2.0});
    laneletTwo->addRightVertex(vertice{12, 1.0});
    laneletTwo->addCenterVertex(vertice{12, 1.5});
    laneletTwo->constructOuterPolygon();
    leftBorderTwo.push_back(vertice{12, 2.0});
    rightBorderTwo.push_back(vertice{12, 1.0});
    centerVerticesTwo.push_back(vertice{12, 1.5});

    //rear lanelet
    idThree = 3;
    laneletTypeThree = std::vector<LaneletType>{LaneletType::urban};
    centerVerticesThree = std::vector<vertice>{vertice{-5, 0.5}, vertice{-4, 0.5}, vertice{-3, 0.5},
                                               vertice{-2, 0.5}, vertice{-1, 0.5}, vertice{0, 0.5}};
    leftBorderThree = std::vector<vertice>{vertice{-5, 1}, vertice{-4, 1}, vertice{-3, 1},
                                           vertice{-2, 1.0}, vertice{-1, 1.0}, vertice{0, 1.0}};
    rightBorderThree = std::vector<vertice>{vertice{-5, 0}, vertice{-4, 0}, vertice{-3, 0},
                                            vertice{-2, 0.0}, vertice{-1, 0.0}, vertice{0, 0.0}};
    Lanelet laneletThreeTmp = Lanelet();
    laneletThreeTmp.setId(idThree);
    laneletThreeTmp.setLeftBorderVertices(leftBorderThree);
    laneletThreeTmp.setRightBorderVertices(rightBorderThree);
    laneletThreeTmp.createCenterVertices();
    laneletThreeTmp.constructOuterPolygon();
    laneletThree = std::make_shared<Lanelet>(laneletThreeTmp);

    //right lanelet
    int idFour{ 4 };
    std::vector<LaneletType> laneletTypeFour{ std::vector<LaneletType>{LaneletType::highway} };
    std::vector<vertice> leftBorderFour{vertice{6, 0.0}, vertice{7, 0.0}, vertice{8, 0.0},
                                        vertice{9, 0.0}, vertice{10, 0.0}, vertice{11, 0.0}};
    std::vector<vertice> rightBorderFour{vertice{6, -1.0}, vertice{7, -1.0}, vertice{8, -1.0},
                                         vertice{9, -1.0}, vertice{10, -1.0}, vertice{11, -1.0}};
    laneletFour = std::make_shared<Lanelet>(Lanelet(idFour, leftBorderFour, rightBorderFour, laneletTypeFour));

    //left lanelet
    int idFive{ 5 };
    std::vector<LaneletType> laneletTypeFive{ std::vector<LaneletType>{LaneletType::urban, LaneletType::crosswalk} };
    std::vector<vertice> leftBorderFive{vertice{6, 2.0}, vertice{7, 2.0}, vertice{8, 2.0},
                                        vertice{9, 2.0}, vertice{10, 2.0}, vertice{11, 2.0}};
    std::vector<vertice> rightBorderFive{vertice{6, 1.0}, vertice{7, 1.0}, vertice{8, 1.0},
                                         vertice{9, 1.0}, vertice{10, 1.0}, vertice{11, 1.0}};
    laneletFive = std::make_shared<Lanelet>(Lanelet(idFive, leftBorderFive, rightBorderFive, laneletTypeFive));

    //second front lanelet
    int idSix{ 6 };
    std::vector<LaneletType> laneletTypeSix{ std::vector<LaneletType>{LaneletType::mainCarriageWay,
                                                                      LaneletType::interstate} };
    std::vector<vertice> leftBorderSix = std::vector<vertice>{vertice{6, 1.0}, vertice{7, 1.0},
                                                              vertice{8, 1.0}};
    std::vector<vertice> rightBorderSix = std::vector<vertice>{vertice{6, 0}, vertice{7, 0},
                                                               vertice{8, 0}};
    laneletSix = std::make_shared<Lanelet>(Lanelet(idSix, leftBorderSix, rightBorderSix, laneletTypeSix));

    //second rear lanelet
    int idSeven{ 7 };
    std::vector<LaneletType> laneletTypeSeven{ std::vector<LaneletType>{LaneletType::urban, LaneletType::crosswalk} };
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
    tl->setCycle(std::vector<CycleElement>{CycleElement{CycleElementType::green, 1.0},
                                           CycleElement{CycleElementType::yellow, 1.0},
                                           CycleElement{CycleElementType::red, 1.0},
                                           CycleElement{CycleElementType::red_yellow, 1.0}});

    StopLine sl = StopLine();
    sl.setLineMarking(LineMarking::broad_solid);
    sl.setPoints(std::vector<vertice>{vertice{1, 2}, vertice{3, 4}});
    sl.setTrafficSign(ts);
    sl.setTrafficLight(tl);

    // add successors, predecessors, adjacent, traffic sign, traffic light, and stop line to lanelet one
    laneletOne->addSuccessor(laneletTwo);
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

    geometry::EigenPolyline reference_path;
    for(auto vert : laneletOne->getCenterVertices()){
        reference_path.push_back(Eigen::Vector2d(vert.x, vert.y));
    }

    laneOne = std::make_shared<Lane>(Lane(std::vector<std::shared_ptr<Lanelet>>{laneletOne},
                                          *laneletOne,
                                          CurvilinearCoordinateSystem(reference_path)));

    std::vector<std::shared_ptr<Lanelet>> lanelets{ laneletOne, laneletTwo, laneletThree, laneletFour, laneletFive};

    roadNetwork = std::make_shared<RoadNetwork>(RoadNetwork(lanelets));

}

void RoadNetworkTest::SetUp(){
    setUpRoadNetwork();
}

TEST_F(RoadNetworkTest, InitializationComplete){

}