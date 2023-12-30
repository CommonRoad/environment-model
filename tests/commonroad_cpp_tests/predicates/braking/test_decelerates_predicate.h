#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/predicates/braking/decelerates_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestDeceleratePredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world;
    DeceleratesPredicate pred;

  private:
    void SetUp() override;
};
