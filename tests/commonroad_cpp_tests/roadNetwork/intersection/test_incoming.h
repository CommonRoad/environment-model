//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include <gtest/gtest.h>

class IncomingTestInitialization {
  protected:
    std::shared_ptr<IncomingGroup> incomingOne;
    std::shared_ptr<IncomingGroup> incomingTwo;
    std::shared_ptr<IncomingGroup> incomingThree;
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    void setUpIncoming();
};

class IncomingTest : public IncomingTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
