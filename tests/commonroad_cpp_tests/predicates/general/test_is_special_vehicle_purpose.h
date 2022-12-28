//
// Created by valentin on 02.11.22.
//

#pragma once

#include "commonroad_cpp/world.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/general/is_special_vehicle_purpose_predicate.h>
#include <gtest/gtest.h>

class IsSpecialVehiclePurposeTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    isSpecialVehiclePurposePredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
