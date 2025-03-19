#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class StopLineInFrontPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world;
    StopLineInFrontPredicate pred;

  private:
    void SetUp() override;
};
