//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "test_incoming.h"

class IntersectionTestInitialization : public IncomingTestInitialization {
  protected:
    void setUpIntersection();
    std::shared_ptr<Intersection> intersection1;
    std::shared_ptr<Intersection> intersection2;
    std::shared_ptr<RoadNetwork> roadNetwork;
};

class IntersectionTest : public IntersectionTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
