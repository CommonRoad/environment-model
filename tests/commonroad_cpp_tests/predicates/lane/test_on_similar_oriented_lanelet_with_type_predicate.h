#pragma once
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_with_type_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class OnSimilarOrientedLaneletWithTypePredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    OnSimilarOrientedLaneletWithTypePredicate pred;
    std::shared_ptr<World> world;
    std::vector<std::string> opt;

    void initializeTestData(const std::string &laneletType1, const std::string &laneletType2);

  private:
    void SetUp() override;
};
