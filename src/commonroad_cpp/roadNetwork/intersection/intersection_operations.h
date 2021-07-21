//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "../../obstacle/obstacle.h"
#include "../road_network.h"

namespace intersection_operations {

/**
 * Evaluates whether obstacle occupies a incoming lanelet.
 * @param timeStep Time step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @return Boolean indicating whether incoming is occupied by obstacle.
 */
bool onIncoming(size_t timeStep, const std::shared_ptr<Obstacle> &obs, const std::shared_ptr<RoadNetwork> &roadNetwork);

} // namespace intersection_operations
