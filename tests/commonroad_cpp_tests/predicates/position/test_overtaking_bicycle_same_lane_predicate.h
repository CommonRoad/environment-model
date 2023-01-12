#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/overtaking_bicycle_same_lane_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestOvertakingBicycleSameLanePredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleEgo;
    std::shared_ptr<Obstacle> obstacleOne;
    OvertakingBicycleSameLanePredicate pred;

  private:
    void SetUp() override;
};
