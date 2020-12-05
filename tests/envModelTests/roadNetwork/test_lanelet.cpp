//
// Created by sebastian on 03.12.20.
//

#include "test_lanelet.h"

void compareVerticesVector(std::vector<vertice> verticesOne, std::vector<vertice> verticesTwo){
    for(int i=0; i<verticesOne.size(); ++i){
        EXPECT_EQ(verticesOne.at(i).x, verticesTwo.at(i).x);
        EXPECT_EQ(verticesOne.at(i).y, verticesTwo.at(i).y);
    }
}

void LaneletTest::SetUp() {
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
    laneletTwo->createCenterVertices();

    //rear lanelet
    idThree = 1;
    laneletTypeThree = std::vector<LaneletType>{LaneletType::urban};
    centerVerticesThree = std::vector<vertice>{vertice{11, .5}, vertice{12, .5}, vertice{13, .5},
                                               vertice{14, 1}, vertice{15, 1.5}, vertice{16, 1.5}};
    leftBorderThree = std::vector<vertice>{vertice{11, 1}, vertice{12, 1}, vertice{13, 1},
                                           vertice{14, 1.5}, vertice{15, 2}, vertice{16, 2}};
    rightBorderThree = std::vector<vertice>{vertice{11, 0}, vertice{12, 0}, vertice{13, 0},
                                            vertice{14, 0.0}, vertice{15, 0.0}, vertice{16, 0.0}};
    laneletThree = Lanelet();

    //right lanelet
    std::vector<vertice> centerVerticesFour{vertice{6, -0.5}, vertice{7, -0.5}, vertice{8, -0.5},
                                            vertice{9, -0.5}, vertice{10, -0.5}, vertice{11, -0.5}};
    std::vector<vertice> leftBorderFour{vertice{6, 0.0}, vertice{7, 0.0}, vertice{8, 0.0},
                                        vertice{9, 0.0}, vertice{10, 0.0}, vertice{11, 0.0}};
    std::vector<vertice> rightBorderFour{vertice{6, -1.5}, vertice{7, -1.5}, vertice{8, -1.5},
                                         vertice{9, -1.5}, vertice{10, -1.5}, vertice{11, -1.5}};
    //left lanelet
    std::vector<vertice> centerVerticesFive{vertice{6, 1.5}, vertice{7, 1.5}, vertice{8, 1.5},
                                            vertice{9, 1.5}, vertice{10, 1.5}, vertice{11, 1.5}};
    std::vector<vertice> leftBorderFive{vertice{6, 2.0}, vertice{7, 2.0}, vertice{8, 2.0},
                                        vertice{9, 2.0}, vertice{10, 2.0}, vertice{11, 2.0}};
    std::vector<vertice> rightBorderFive{vertice{6, 1.0}, vertice{7, 1.0}, vertice{8, 1.0},
                                         vertice{9, 1.0}, vertice{10, 1.0}, vertice{11, 1.0}};

    laneletOne->addSuccessor(laneletTwo);
};


TEST_F(LaneletTest, InitializationComplete){
    //middle lanelet

    compareVerticesVector(laneletOne->getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne->getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne->getRightBorderVertices(), rightBorderOne);
    EXPECT_EQ(laneletOne->getId(), idOne);

//    compareVerticesVector(laneletTwo.getCenterVertices(), centerVerticesTwo);
//    compareVerticesVector(laneletTwo.getLeftBorderVertices(), centerVerticesTwo);
//    compareVerticesVector(laneletTwo.getRightBorderVertices(), centerVerticesTwo);
//
//    compareVerticesVector(laneletThree.getCenterVertices(), centerVerticesThree);
//    compareVerticesVector(laneletThree.getLeftBorderVertices(), leftBorderThree);
//    compareVerticesVector(laneletThree.getRightBorderVertices(), rightBorderThree);
//
//    compareVerticesVector(laneletOne.getCenterVertices(), centerVerticesOne);
//    compareVerticesVector(laneletOne.getLeftBorderVertices(), leftBorderOne);
//    compareVerticesVector(laneletOne.getRightBorderVertices(), rightBorderOne);
//
//    compareVerticesVector(laneletOne.getCenterVertices(), centerVerticesOne);
//    compareVerticesVector(laneletOne.getLeftBorderVertices(), leftBorderOne);
//    compareVerticesVector(laneletOne.getRightBorderVertices(), rightBorderOne);



}