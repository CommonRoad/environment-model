//
// Created by wilhelm on 6/15/21.
//

#ifndef ENV_MODEL_TRAFFICSIGNINTERPRETERTEST_H
#define ENV_MODEL_TRAFFICSIGNINTERPRETERTEST_H
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <gtest/gtest.h>
class TrafficSignInterpreterTest : public testing::Test {

  protected:
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns2020a;
    std::vector<std::shared_ptr<TrafficLight>> trafficLights2020a;
    std::vector<std::shared_ptr<Lanelet>> lanelets2020a;
    std::vector<std::shared_ptr<Obstacle>> obstacles2020a;
    std::vector<std::shared_ptr<Intersection>> intersections2020a;
    std::shared_ptr<RoadNetwork> roadNetwork2020a;

    std::vector<std::shared_ptr<TrafficSign>> trafficSigns2018b;
    std::vector<std::shared_ptr<TrafficLight>> trafficLights2018b;
    std::vector<std::shared_ptr<Lanelet>> lanelets2018b;
    std::vector<std::shared_ptr<Obstacle>> obstacles2018b;
    std::vector<std::shared_ptr<Intersection>> intersections2018b;
    std::shared_ptr<RoadNetwork> roadNetwork2018b;

    void SetUp() override;
};

#endif // ENV_MODEL_TRAFFICSIGNINTERPRETERTEST_H
