#pragma once
#include "../../interfaces/utility_functions.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/on_incoming_left_of_predicate.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
#include <gtest/gtest.h>

class OnIncomingLeftOfPredicateTest : public testing::Test {
  protected:
    OnIncomingLeftOfPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
