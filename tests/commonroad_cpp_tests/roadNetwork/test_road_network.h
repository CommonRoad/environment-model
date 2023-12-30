//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include <memory>

#include "intersection/test_intersection.h"
#include "lanelet/test_lane.h"

class RoadNetwork;

class RoadNetworkTestInitialization : public LaneTestInitialization, public IntersectionTestInitialization {
  protected:
    std::shared_ptr<RoadNetwork> roadNetwork;
    void setUpRoadNetwork();
};

class RoadNetworkTest : public RoadNetworkTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
