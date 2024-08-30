#pragma once
#include "../../interfaces/utility_functions.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/on_oncoming_of_predicate.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
#include <gtest/gtest.h>

class OnOncomingOfPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    OnOncomingOfPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
