#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/in_neighboring_lane_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestInNeighboringLanePredicate : public testing::Test {
  protected:
    void SetUpRightSide();
    std::shared_ptr<Obstacle> obstacleEgo;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    std::shared_ptr<Obstacle> obstacleFive;
    std::shared_ptr<Obstacle> obstacleSix;
    InNeighboringLanePredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<World> worldOncoming;

  private:
    void SetUp() override;
};
