//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "obstacle_operations.h"

std::shared_ptr<Obstacle>
obstacle_operations::getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList, size_t id) {
    std::shared_ptr<Obstacle> temp{nullptr};
    for (auto &obs : obstacleList) {
        if (obs->getId() == id) {
            temp = obs;
            break;
        }
    }
    return temp;
}

ObstacleType obstacle_operations::matchStringToObstacleType(const std::string &type) {
    if (type == "car")
        return ObstacleType::car;
    else if (type == "truck")
        return ObstacleType::truck;
    else if (type == "pedestrian")
        return ObstacleType::pedestrian;
    else if (type == "bus")
        return ObstacleType::bus;
    else if (type == "vehicle")
        return ObstacleType::vehicle;
    else
        return ObstacleType::unknown;
}
