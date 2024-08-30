#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_without_type_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class OnSimilarOrientedLaneletWithoutTypePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    OnSimilarOrientedLaneletWithoutTypePredicate pred;
    std::shared_ptr<World> world;
    std::vector<std::string> opt;

  private:
    void SetUp() override;
};
