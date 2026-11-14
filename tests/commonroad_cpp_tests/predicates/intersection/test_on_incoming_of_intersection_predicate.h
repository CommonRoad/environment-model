#pragma once
#include <commonroad_cpp/predicates/intersection/on_incoming_of_intersection_predicate.h>
#include <commonroad_cpp/world.h>
#include <gtest/gtest.h>

class OnIncomingOfIntersectionPredicateTest : public testing::Test {
  protected:
    OnIncomingOfIntersectionPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
