#pragma once

#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include <gtest/gtest.h>

class IntersectionTestInitialization {
  protected:
    std::shared_ptr<IncomingGroup> incomingOne;
    std::shared_ptr<IncomingGroup> incomingTwo;
    std::shared_ptr<IncomingGroup> incomingThree;
    std::shared_ptr<Intersection> intersection1;
    std::shared_ptr<Intersection> intersection2;
    std::shared_ptr<RoadNetwork> roadNetwork;
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    void setUpIncoming();
};

class IncomingTest : public IntersectionTestInitialization, public testing::Test {
  private:
    void SetUp() override;
};
