//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "test_incoming.h"

class IntersectionTest : public IntersectionTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
