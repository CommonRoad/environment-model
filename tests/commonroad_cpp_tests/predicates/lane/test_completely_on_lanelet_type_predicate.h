#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/completely_on_lanelet_type_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class CompletelyOnLaneletTypePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    CompletelyOnLaneletTypePredicate pred;
    std::shared_ptr<World> world1;
    std::shared_ptr<World> world2;

  private:
    void SetUp() override;
};
