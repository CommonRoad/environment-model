#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/in_single_lane_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestInSingleLanePredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    InSingleLanePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
