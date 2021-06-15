//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#pragma once

#include "../../obstacle/obstacle.h"
#include "../road_network.h"

namespace regulatory_elements_utils {

/**
 * Returns all active traffic lights relevant for an obstacle.
 *
 * @param timeStep Time Step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @return Set of pointers to traffic lights.
 */
std::set<std::shared_ptr<TrafficLight>> activeTrafficLights(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                                            std::shared_ptr<RoadNetwork> roadNetwork);

/**
 * Evaluates whether obstacle is at red traffic light for defined turning direction.
 *
 * @param timeStep Time Step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @param turnDir Turning direction which should be considered.
 * @return Boolean indicating whether obstacle is at red traffic light.
 */
bool atRedTrafficLight(size_t timeStep, const std::shared_ptr<Obstacle> &obs, std::shared_ptr<RoadNetwork> roadNetwork,
                       TurningDirections turnDir);

}; // namespace regulatory_elements_utils
