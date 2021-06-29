//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#pragma once

#include "obstacle.h"

namespace obstacle_operations {

/**
 * Returns the obstacle which corresponds to a given obstacle ID.
 *
 * @param id obstacle ID
 * @return pointer to obstacle
 */
std::shared_ptr<Obstacle> getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList, size_t id);

/**
 * Matches a string to an obstacle type
 *
 * @param string for which obstacle type should be extracted
 * @return obstacle type which corresponds to string or unknown type if string does not match
 */
ObstacleType matchStringToObstacleType(const std::string& type);

}