//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "obstacle_operations.h"

std::shared_ptr<Obstacle> getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList, int id) {
    std::shared_ptr<Obstacle> temp{nullptr};
    for (auto &obs : obstacleList) {
        if (obs->getId() == id) {
            temp = obs;
            break;
        }
    }
    return temp;
}

ObstacleType matchObstacleTypeToString(const char *type) {
    if (!(strcmp(type, "car")))
        return ObstacleType::car;
    else if (!(strcmp(type, "truck")))
        return ObstacleType::truck;
    else if (!(strcmp(type, "pedestrian")))
        return ObstacleType::pedestrian;
    else if (!(strcmp(type, "bus")))
        return ObstacleType::bus;
    else if (!(strcmp(type, "vehicle")))
        return ObstacleType::vehicle;
    else
        return ObstacleType::unknown;
}

