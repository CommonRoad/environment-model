#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/general/change_lane_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class ChangeLanePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    std::shared_ptr<Obstacle> obstacleFour;
    ChangeLanePredicate pred;
    std::shared_ptr<World> world1;
    std::shared_ptr<World> world2;
    std::string pathToTestFileAccessRamp;
    std::string pathToTestFileExitRamp;
    std::string pathToTestFileTwoFollowingLaneletsAndAccessRampFalse;
    std::string pathToTestFileTwoFollowingLaneletsAndAccessRampTrue;
    std::string pathToTestFileOvertakingOncomingLanelet;

    std::vector<std::string> optLeft;
    std::vector<std::string> optRight;

  private:
    void SetUp() override;
};
