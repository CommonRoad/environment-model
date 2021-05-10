//
// Created by sebastian on 08.12.20.
//

#ifndef ENV_MODEL_TEST_ROAD_NETWORK_H
#define ENV_MODEL_TEST_ROAD_NETWORK_H

#include "commonroad_cpp/roadNetwork/road_network.h"
#include "lanelet/test_lane.h"

class RoadNetworkTestInitialization : public LaneTestInitialization {
  protected:
    std::shared_ptr<RoadNetwork> roadNetwork;
    void setUpRoadNetwork();
};

class RoadNetworkTest : public RoadNetworkTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_ROAD_NETWORK_H
