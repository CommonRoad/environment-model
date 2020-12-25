//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_OBSTACLE_OPERATIONS_H
#define ENV_MODEL_OBSTACLE_OPERATIONS_H

#include "obstacle.h"

// find Obstacle by id
std::shared_ptr<Obstacle> getObstacleById(const std::vector<std::shared_ptr<Obstacle>>& obstacleList, int id);

ObstacleType matchObstacleTypeToString(const char *type);

#endif //ENV_MODEL_OBSTACLE_OPERATIONS_H
