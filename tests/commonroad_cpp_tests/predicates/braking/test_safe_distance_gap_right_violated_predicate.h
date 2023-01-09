#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/braking/safe_distance_gap_right_violated_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class TestSafeDistanceGapRightViolatedPredicate : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleEgo;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    std::shared_ptr<Obstacle> obstacleFive;
    SafeDistanceGapRightViolatedPredicate pred;
    std::shared_ptr<World> world;
    std::string pathToTestFile;
    std::string pathToTestFileOvertaking;

  private:
    void SetUp() override;
};
