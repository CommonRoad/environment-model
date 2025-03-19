#pragma once

#include "../test_road_network.h"

class LaneletOperationsTest : public ::RoadNetworkTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
