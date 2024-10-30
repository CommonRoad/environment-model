#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/intersection/at_intersection_type_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class AtIntersectionTypePredicateTest : public testing::Test {
  protected:
    void SetUpTIntersection();
    void SetUpUncontrolledIntersection();
    std::shared_ptr<World> world;
    std::shared_ptr<World> world_2;
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    std::shared_ptr<Obstacle> obstacleTwo;
    AtIntersectionTypePredicate pred;

  private:
    void SetUp() override;
};
