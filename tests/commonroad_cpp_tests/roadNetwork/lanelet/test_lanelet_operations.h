//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "../test_road_network.h"

class LaneletOperationsTest : public ::RoadNetworkTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
