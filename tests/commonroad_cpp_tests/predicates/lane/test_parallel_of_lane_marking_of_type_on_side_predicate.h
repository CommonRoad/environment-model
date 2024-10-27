#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/parallel_to_lane_marking_of_type_on_side_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class ParallelOfLaneMarkingOfTypeOnSidePredicateTest : public testing::Test {
  protected:
    void SetUpRightSide();
    std::shared_ptr<Obstacle> egoVehicle;
    ParallelToLaneMarkingOfTypeOnSidePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
