#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/regulatory/relevant_traffic_light_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class RelevantRedTrafficLightPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world;
    RelevantTrafficLightPredicate pred;
};
