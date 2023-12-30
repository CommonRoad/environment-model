//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once
#include "../../roadNetwork/intersection/test_intersection.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/position/unobstructed_intersection_view_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class UnobstructedIntersectionViewPredicateTest : public testing::Test, public IntersectionTestInitialization {
  protected:
    std::shared_ptr<Obstacle> egoVehicle;
    UnobstructedIntersectionViewPredicate pred;
    std::shared_ptr<World> world;

  private:
    void SetUp() override;
};
