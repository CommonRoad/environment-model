//
// Created by sebastian on 07.12.20.
//

#ifndef ENV_MODEL_TEST_LANELET_OPERATIONS_H
#define ENV_MODEL_TEST_LANELET_OPERATIONS_H

#include "../test_road_network.h"

class LaneletOperationsTest : public ::RoadNetworkTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};

#endif // ENV_MODEL_TEST_LANELET_OPERATIONS_H
