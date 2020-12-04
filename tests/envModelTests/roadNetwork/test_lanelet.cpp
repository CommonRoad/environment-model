//
// Created by sebastian on 03.12.20.
//

#include "test_lanelet.h"
#include "roadNetwork/lanelet/lanelet.h"

TEST(TestLanlet, initialization){
    size_t id{5};
    std::vector<vertice> centerVerticesOne{vertice{0, .5}, vertice{1, .5}, vertice{2, .5},
                                        vertice{3, 1}, vertice{4, 1.5}, vertice{5, 1.5}};
    std::vector<vertice> leftBorderOne{vertice{0, 1}, vertice{1, 1}, vertice{2, 1},
                                    vertice{3, 1.5}, vertice{4, 2}, vertice{5, 2}};
    std::vector<vertice> rightBorderOne{vertice{0, 0}, vertice{1, 0}, vertice{2, 0},
                                     vertice{3, .5}, vertice{4, 1}, vertice{5, 1}};
    std::vector<std::shared_ptr<Lanelet>> predecessorLaneletsOne;
    std::vector<std::shared_ptr<Lanelet>> successorLaneletsOne;
    std::vector<std::shared_ptr<TrafficLight>> trafficLights;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;
    std::vector<LaneletType> laneletType;
    std::vector<ObstacleType> userOneWay;
    std::vector<ObstacleType> userBidirectional;
    Lanelet l1 = Lanelet();

}