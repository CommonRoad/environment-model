#pragma once

#include <commonroad_cpp/predicates/position/lateral_distance_to_obstacle_above_threshold_predicate.h>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <gtest/gtest.h>
#include <memory>

class TestLateralDistanceToObstacleAboveThresholdPredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne2;
    std::shared_ptr<Obstacle> egoVehicle2;
    std::shared_ptr<World> world;
    std::shared_ptr<World> world2;
    LateralDistanceToObstacleAboveThresholdPredicate pred;
    const std::string dMinNonUrban{"2.0"};
    const std::string dMinUrban{"1.5"};
    const std::string closeToOtherVehicle{"0.5"};

    void setUpLeft();
    void setUpRight();

  private:
    void SetUp() override;
};
