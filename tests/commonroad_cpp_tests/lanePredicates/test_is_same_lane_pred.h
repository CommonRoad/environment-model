#pragma once

#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/lanePredicates/is_same_lane_pred.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestIsSameLanePredicate : public testing::Test {
  protected:
    std::shared_ptr<Lane> laneOne;
    std::shared_ptr<Lane> laneTwo;
    std::shared_ptr<Lane> laneThree;
    IsSameLanePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
