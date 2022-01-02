//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "commonroad_cpp/roadNetwork/intersection/incoming.h"
#include <gtest/gtest.h>

class IncomingTestInitialization {
  protected:
    std::unique_ptr<Incoming> incomingOne;
    std::unique_ptr<Incoming> incomingTwo;
    std::unique_ptr<Incoming> incomingThree;
    void setUpIncoming();
};

class IncomingTest : public IncomingTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
