//
// Created by valentin on 04.11.22.
//

#pragma once

#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/general/right_signal_set_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class RightSignalSetPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    RightSignalSetPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
