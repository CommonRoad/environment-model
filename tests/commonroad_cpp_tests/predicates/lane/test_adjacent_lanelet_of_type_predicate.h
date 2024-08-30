#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/adjacent_lanelet_of_type_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class AdjacentLaneletOfTypePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    std::shared_ptr<Obstacle> obstacleThree;
    AdjacentLaneletOfTypePredicate pred;
    std::shared_ptr<World> worldOne;
    std::shared_ptr<World> worldTwo;
    std::shared_ptr<World> worldThree;

  private:
    void SetUp() override;
};
