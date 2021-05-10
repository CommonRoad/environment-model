//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_OBSTACLE_OPERATIONS_H
#define ENV_MODEL_OBSTACLE_OPERATIONS_H

#include "obstacle.h"

/**
 * Returns the obstacle which corresponds to a given obstacle ID.
 *
 * @param id obstacle ID
 * @return pointer to obstacle
 */
std::shared_ptr<Obstacle> getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList, int id);

/**
 * Matches a string to an obstacle type
 *
 * @param string for which obstacle type should be extracted
 * @return obstacle type which corresponds to string or unknown type if string does not match
 */
ObstacleType matchStringToObstacleType(std::string type);

#endif // ENV_MODEL_OBSTACLE_OPERATIONS_H
