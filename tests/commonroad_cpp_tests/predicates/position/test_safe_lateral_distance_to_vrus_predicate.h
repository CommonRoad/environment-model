#pragma once

#include <commonroad_cpp/predicates/position/safe_lateral_distance_to_vrus_predicate.h>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <gtest/gtest.h>
#include <memory>

class TestSafeDistanceToVrusPredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> egoVehicle;
    SafeLateralDistanceToVrusPredicate pred;
    const std::string dMinNonUrban{"2.0"};
    const std::string dMinUrban{"1.5"};

  private:
    void SetUp() override;
};
