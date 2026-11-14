#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/regulatory/at_traffic_light_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class AtRedTrafficLightPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world;
    AtTrafficLightPredicate pred;
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    std::shared_ptr<RoadNetwork> roadNetwork;
    double timeStepSize;

  private:
    void SetUp() override;
};
