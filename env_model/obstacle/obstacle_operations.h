/*
 * functions on obstacles
 */

#ifndef HEADER_OBSTACLE_OPERATIONS
#define HEADER_OBSTACLE_OPERATIONS

#include "obstacle.h"

// find Obstacle by id
Obstacle *getObstacleById(std::vector<Obstacle *> *obstacleList, size_t id);

// find Obstacle by id and set pointer to zero
Obstacle *getObstacleByIdAndRemove(std::vector<Obstacle *> *obstacleList, size_t id);

// returns true if Obstacle id is already present in previous obstacles
bool obstacleInPreviousObstacles(std::vector<Obstacle *> *obstacleList, size_t id);

//bool isObstacleTypeOfClass(const std::shared_ptr<Obstacle> &Obstacle, const std::string &ClassName);



#endif
