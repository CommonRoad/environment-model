//
// Created by sebastian on 07.12.20.
//

#ifndef ENV_MODEL_TEST_LANE_H
#define ENV_MODEL_TEST_LANE_H

#include <gtest/gtest.h>
#include "../test_road_network.h"
#include "roadNetwork/lanelet/lane.h"

class LaneTest : public::RoadNetworkTest{
protected:
    std::shared_ptr<Lane> laneOne;
private:
    void SetUp() override;
};

#endif //ENV_MODEL_TEST_LANE_H
