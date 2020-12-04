//
// Created by sebastian on 03.12.20.
//

#include "test_lanelet.h"
#include "roadNetwork/lanelet/lanelet.h"

void compareVerticesVector(std::vector<vertice> verticesOne, std::vector<vertice> verticesTwo){
    for(int i=0; i<verticesOne.size(); ++i){
        EXPECT_EQ(verticesOne.at(i).x, verticesTwo.at(i).x);
        EXPECT_EQ(verticesOne.at(i).y, verticesTwo.at(i).y);
    }
}

TEST(TestLanelet, Initialization1){
    //middle lanelet
    int idOne{1};
    std::vector<LaneletType> laneletTypeOne{LaneletType::mainCarriageWay, LaneletType::urban};
    std::vector<ObstacleType> userOneWayOne{ObstacleType::car, ObstacleType::bus};
    std::vector<ObstacleType> userBidirectionalOne{ObstacleType::truck, ObstacleType::pedestrian};
    std::vector<vertice> centerVerticesOne{vertice{0, 0.5}, vertice{1, 0.5}, vertice{2, 0.5},
                                        vertice{3, 0.5}, vertice{4, 0.5}, vertice{5, 0.5}};
    std::vector<vertice> leftBorderOne{vertice{0, 1}, vertice{1, 1}, vertice{2, 1},
                                    vertice{3, 1.0}, vertice{4, 1.0}, vertice{5, 1.0}};
    std::vector<vertice> rightBorderOne{vertice{0, 0}, vertice{1, 0}, vertice{2, 0},
                                     vertice{3, 0}, vertice{4, 0}, vertice{5, 0}};
    //front lanelet
    std::vector<vertice> centerVerticesTwo{vertice{6, 0.5}, vertice{7, 0.5}, vertice{8, 0.5},
                                           vertice{9, 0.5}, vertice{10, 0.5}, vertice{11, 0.5}};
    std::vector<vertice> leftBorderTwo{vertice{6, 1.0}, vertice{7, 1.0}, vertice{8, 1.0},
                                       vertice{9, 1.0}, vertice{10, 1.0}, vertice{11, 1.0}};
    std::vector<vertice> rightBorderTwo{vertice{6, 0}, vertice{7, 0}, vertice{8, 0},
                                        vertice{9, 0.0}, vertice{10, 0.0}, vertice{11, 0.0}};
    //rear lanelet
    std::vector<vertice> centerVerticesThree{vertice{11, .5}, vertice{12, .5}, vertice{13, .5},
                                           vertice{14, 1}, vertice{15, 1.5}, vertice{16, 1.5}};
    std::vector<vertice> leftBorderThree{vertice{11, 1}, vertice{12, 1}, vertice{13, 1},
                                       vertice{14, 1.5}, vertice{15, 2}, vertice{16, 2}};
    std::vector<vertice> rightBorderThree{vertice{11, 0}, vertice{12, 0}, vertice{13, 0},
                                        vertice{14, 0.0}, vertice{15, 0.0}, vertice{16, 0.0}};
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
    Lanelet laneletOne = Lanelet(idOne, centerVerticesOne, leftBorderOne, rightBorderOne, laneletTypeOne, userOneWayOne,
                                 userBidirectionalOne);

    compareVerticesVector(laneletOne.getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne.getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne.getRightBorderVertices(), rightBorderOne);

    compareVerticesVector(laneletOne.getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne.getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne.getRightBorderVertices(), rightBorderOne);

    compareVerticesVector(laneletOne.getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne.getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne.getRightBorderVertices(), rightBorderOne);

    compareVerticesVector(laneletOne.getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne.getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne.getRightBorderVertices(), rightBorderOne);

    compareVerticesVector(laneletOne.getCenterVertices(), centerVerticesOne);
    compareVerticesVector(laneletOne.getLeftBorderVertices(), leftBorderOne);
    compareVerticesVector(laneletOne.getRightBorderVertices(), rightBorderOne);



    std::vector<std::shared_ptr<Lanelet>> predecessorLaneletsOne;
    std::vector<std::shared_ptr<Lanelet>> successorLaneletsOne;
    std::vector<std::shared_ptr<TrafficLight>> trafficLights;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;




}