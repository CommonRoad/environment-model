#pragma once
#include "../../roadNetwork/intersection/test_intersection.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/intersection/in_intersection_conflict_area_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class InIntersectionConflictAreaPredicateTest : public testing::Test, public IntersectionTestInitialization {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> obstacleOne;
    InIntersectionConflictAreaPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
