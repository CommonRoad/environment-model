//
// Created by sebastian on 03.12.20.
//

#include "test_lanelet.h"

#include "roadNetwork/regulatory_elements/stop_line.h"
#include "roadNetwork/regulatory_elements/traffic_sign_element.h"
#include "roadNetwork/regulatory_elements/traffic_sign.h"
#include "roadNetwork/regulatory_elements/traffic_light.h"


void compareVerticesVector(std::vector<vertice> verticesOne, std::vector<vertice> verticesTwo){
    for(int i=0; i<verticesOne.size(); ++i){
        EXPECT_EQ(verticesOne.at(i).x, verticesTwo.at(i).x);
        EXPECT_EQ(verticesOne.at(i).y, verticesTwo.at(i).y);
    }
}

template <typename T>
void evaluateTypes(std::vector<T> typeVectorOne, std::vector<T> typeVectorTwo) {
    for(auto ty : typeVectorOne){
        bool typeValid{ false };
        for(auto typeExpected : typeVectorTwo)
            if(ty == typeExpected){
                typeValid = true;
                break;
            }
        EXPECT_EQ(typeValid, true);
    }
}

void LaneletTest::SetUp() {
    //middle lanelet
    idOne = 1;
    laneletTypeOne = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::urban};
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
    std::vector<vertice> rightBorderFour{vertice{6, -1.5}, vertice{7, -1.5}, vertice{8, -1.5},
                                         vertice{9, -1.5}, vertice{10, -1.5}, vertice{11, -1.5}};
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
    std::vector<LaneletType> laneletTypeSix{ std::vector<LaneletType>{LaneletType::highway} };
    std::vector<vertice> leftBorderSix = std::vector<vertice>{vertice{6, 1.0}, vertice{7, 1.0}, vertice{8, 1.0}};
    std::vector<vertice> rightBorderSix = std::vector<vertice>{vertice{6, 0}, vertice{7, 0}, vertice{8, 0}};
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

    laneletOne->addSuccessor(laneletTwo);
    laneletOne->addSuccessor(laneletSix);
    laneletOne->addPredecessor(laneletThree);
    laneletOne->addPredecessor(laneletSeven);
    laneletOne->setLeftAdjacent(laneletFive, DrivingDirection::opposite);
    laneletOne->setRightAdjacent(laneletFour, DrivingDirection::same);
    laneletOne->addTrafficLight(tl);
    laneletOne->addTrafficSign(ts);
    laneletOne->setStopLine(sl);
};


TEST_F(LaneletTest, InitializationComplete){
    //middle lanelet
    compareVerticesVector(laneletOne->getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne->getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne->getRightBorderVertices(), rightBorderOne);
    EXPECT_EQ(laneletOne->getId(), idOne);
    evaluateTypes(laneletOne->getLaneletType(), laneletTypeOne);
    evaluateTypes(laneletOne->getUserOneWay(), userOneWayOne);
    evaluateTypes(laneletOne->getUserBidirectional(), userBidirectionalOne);
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
    evaluateTypes(laneletTwo->getLaneletType(), laneletTypeTwo);
    evaluateTypes(laneletTwo->getUserOneWay(), userOneWayTwo);
    evaluateTypes(laneletTwo->getUserBidirectional(), userBidirectionalTwo);
    compareVerticesVector(laneletTwo->getCenterVertices(), centerVerticesTwo);
    compareVerticesVector(laneletTwo->getLeftBorderVertices(), leftBorderTwo);
    compareVerticesVector(laneletTwo->getRightBorderVertices(), rightBorderTwo);
}

TEST_F(LaneletTest, InitializationEnd){
    //rear lanelet
    EXPECT_EQ(laneletThree->getId(), idThree);
    evaluateTypes(laneletThree->getLaneletType(), laneletTypeThree);
    compareVerticesVector(laneletThree->getCenterVertices(), centerVerticesThree);
    compareVerticesVector(laneletThree->getLeftBorderVertices(), leftBorderThree);
    compareVerticesVector(laneletThree->getRightBorderVertices(), rightBorderThree);
}

TEST_F(LaneletTest, ApplyIntersectionTesting){

}

TEST_F(LaneletTest, CheckIntersection){

}

TEST_F(LaneletTest, ConstructOuterPolygon){

}

TEST_F(LaneletTest, GetOrientationAtPosition){

}