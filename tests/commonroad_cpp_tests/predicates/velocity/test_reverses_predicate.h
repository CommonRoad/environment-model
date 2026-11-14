#pragma once
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/velocity/reverses_predicate.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class ReversesPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    ReversesPredicate pred;
    std::shared_ptr<World> world;
    PredicateParameters parameters;

  private:
    void SetUp() override;
};
