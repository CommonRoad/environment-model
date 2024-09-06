#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/on_same_road_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestOnSameRoadPredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleEgo;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    std::shared_ptr<Obstacle> obstacleFive;
    std::shared_ptr<Obstacle> obstacleSix;
    OnSameRoadPredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<World> worldOncoming;

  private:
    void SetUp() override;
};
