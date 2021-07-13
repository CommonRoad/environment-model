//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

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
