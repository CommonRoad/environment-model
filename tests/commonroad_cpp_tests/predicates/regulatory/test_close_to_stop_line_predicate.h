#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/regulatory/close_to_stop_line_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class CloseToStopLinePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<World> world;
    CloseToStopLinePredicate pred;

  private:
    void SetUp() override;
};
