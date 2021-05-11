//
// Created by sebastian on 07.12.20.
//

#ifndef ENV_MODEL_TEST_LANE_H
#define ENV_MODEL_TEST_LANE_H

#include "../lanelet/test_lanelet.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"

class LaneTestInitialization : public LaneletTestInitialization {
  protected:
    std::shared_ptr<Lane> laneOne;
    void setUpLane();
};

class LaneTest : public LaneTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_LANE_H
