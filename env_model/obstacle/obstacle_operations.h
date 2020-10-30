/*
 * functions on obstacles
 */

#ifndef HEADER_OBSTACLE_OPERATIONS
#define HEADER_OBSTACLE_OPERATIONS

#include "obstacle.h"

// find obstacle by id
obstacle *getObstacleById(std::vector<obstacle *> *obstacleList, size_t id);

// find obstacle by id and set pointer to zero
obstacle *getObstacleByIdAndRemove(std::vector<obstacle *> *obstacleList, size_t id);

// returns true if obstacle id is already present in previous obstacles
bool obstacleInPreviousObstacles(std::vector<obstacle *> *obstacleList, size_t id);

bool isObstacleTypeOfClass(const std::shared_ptr<obstacle> &Obstacle, const std::string &ClassName);

// not implemented yet
void obstacleInteraction(std::vector<obstacle *> obstacles);

#endif
