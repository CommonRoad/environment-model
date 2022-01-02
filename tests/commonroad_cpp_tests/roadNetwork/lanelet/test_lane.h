//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include <memory>

#include "../lanelet/test_lanelet.h"

class Lane;

class LaneTestInitialization : public LaneletTestInitialization {
  protected:
    std::shared_ptr<Lane> laneOne;
    std::shared_ptr<Lane> laneTwo;
    std::shared_ptr<Lane> laneThree;
    void setUpLane();
};

class LaneTest : public LaneTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
