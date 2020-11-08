//
// Created by Sebastian Maierhofer on 01.11.20.
//

#include "obstacle_operations.h"
#include "../interfaces/commonroad/pugi_xml/pugixml.hpp"

Obstacle *getObstacleById(std::vector<Obstacle *> *obstacleList, size_t id) {
    Obstacle *temp = nullptr;
    for (auto & i : *obstacleList) {
        if (i->getId() == id) {
            temp = i;
            break;
        }
    }
    return temp;
}

ObstacleType matchObstacleTypeToString(const char *type){
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
