#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/narrow_road_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class NarrowRoadPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    std::shared_ptr<Obstacle> egoVehicle2;
    std::shared_ptr<Obstacle> egoVehicle3;
    std::shared_ptr<Obstacle> egoVehicle4;
    std::shared_ptr<Obstacle> egoVehicle5;
    std::shared_ptr<Obstacle> egoVehicle6;
    NarrowRoadPredicate pred;
    std::shared_ptr<World> world;
    std::shared_ptr<World> world2;
    std::shared_ptr<World> world3;
    std::shared_ptr<World> world4;
    std::shared_ptr<World> worldOncoming;
    std::shared_ptr<World> worldOncomingNarrow;

  private:
    void SetUp() override;
};
