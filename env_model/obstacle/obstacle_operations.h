//
// Created by Sebastian Maierhofer on 01.11.20.
//

#ifndef ENV_MODEL_OBSTACLE_OPERATIONS_H
#define ENV_MODEL_OBSTACLE_OPERATIONS_H

#include "obstacle.h"

// find Obstacle by id
Obstacle *getObstacleById(std::vector<Obstacle *> *obstacleList, size_t id);

ObstacleType matchObstacleTypeToString(const char *type);

#endif //ENV_MODEL_OBSTACLE_OPERATIONS_H
