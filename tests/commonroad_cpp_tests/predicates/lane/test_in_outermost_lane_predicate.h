#pragma once
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/in_outermost_lane_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class InOutermostLanePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    InOutermostLanePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
