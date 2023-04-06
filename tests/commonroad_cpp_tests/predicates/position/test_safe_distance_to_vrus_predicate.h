#pragma once

#include "commonroad_cpp/predicates/position/safe_lateral_distance_to_vrus_predicate.h"

class TestSafeDistanceToVrusPredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> egoVehicle;
    SafeLateralDistanceToVrusPredicate pred;

  private:
    void SetUp() override;
};
