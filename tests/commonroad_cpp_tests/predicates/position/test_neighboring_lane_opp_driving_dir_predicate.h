#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/neighboring_lane_opp_driving_dir_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestNeighboringLaneOppDrivingDirPredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleEgo;
    std::shared_ptr<Obstacle> obstacleOne;
    NeighboringLaneOppDrivingDirPredicate pred;
    std::string pathToTestFile;
    std::string pathToTestFileOncoming;
    void initObstacles();

  private:
    void SetUp() override;
};
