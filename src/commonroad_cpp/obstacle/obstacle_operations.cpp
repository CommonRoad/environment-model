//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "obstacle_operations.h"

std::shared_ptr<Obstacle>
obstacle_operations::getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList, size_t obstacleId) {
    std::shared_ptr<Obstacle> temp{nullptr};
    for (const auto &obs : obstacleList) {
        if (obs->getId() == obstacleId) {
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

std::shared_ptr<Obstacle> obstacle_operations::vehicleDirectlyLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                                   const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_left = vehiclesLeft(timeStep, obstacles, obstacleK);
    if (vehicles_left.empty())
        return nullptr;
    else if (vehicles_left.size() == 1)
        return vehicles_left[0];
    else {
        std::shared_ptr<Obstacle> vehicle_directly_left = vehicles_left[0];
        for (const auto &obs : vehicles_left) {
            // What's lat.d
            if (obs->getLatPosition(timeStep) < vehicle_directly_left->getLatPosition(timeStep)) {
                vehicle_directly_left = obs;
            }
        }
        return vehicle_directly_left;
    }
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::vehiclesLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                  const std::shared_ptr<Obstacle> &obstacleK) {

    std::vector<std::shared_ptr<Obstacle>> vehicles_left;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj = vehiclesAdjacent(timeStep, obstacles, obstacleK);

    for (const auto &obs : vehicles_adj) {

        if (obs->rightD(timeStep) > obstacleK->leftD(timeStep)) {
            vehicles_left.push_back(obs);
        }
    }

    return vehicles_left;
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::vehiclesAdjacent(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                      const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehiclesAdj;
    std::vector<std::shared_ptr<Obstacle>> otherVehicles;

    for (const auto &obs : obstacles) {
        if (obs != obstacleK) {
            otherVehicles.push_back(obs);
        }
    }

    for (const auto &obs : otherVehicles) {

        if (!static_cast<bool>(obs->getLonPosition(timeStep)))
            continue;
        if (static_cast<double>(obs->rearS(timeStep) < obstacleK->frontS(timeStep)) < obs->frontS(timeStep)) {
            vehiclesAdj.push_back(obs);
            continue;
        }
        if (static_cast<double>(obs->rearS(timeStep) < obstacleK->rearS(timeStep)) < obs->frontS(timeStep)) {
            vehiclesAdj.push_back(obs);
            continue;
        }
        if (obstacleK->rearS(timeStep) <= obs->rearS(timeStep) and
            obs->frontS(timeStep) <= obstacleK->frontS(timeStep)) {
            vehiclesAdj.push_back(obs);
            continue;
        }
    }

    return otherVehicles;
}

std::shared_ptr<Obstacle>
obstacle_operations::vehicleDirectlyRight(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                          const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_right = vehiclesRight(timeStep, obstacles, obstacleK);
    if (vehicles_right.empty())
        return nullptr;
    else if (vehicles_right.size() == 1)
        return vehicles_right[0];
    else {
        std::shared_ptr<Obstacle> vehicle_directly_right = vehicles_right[0];
        for (const auto &obs : vehicles_right) {
            if (obs->getLatPosition(timeStep) < vehicle_directly_right->getLatPosition(timeStep)) {
                vehicle_directly_right = obs;
            }
        }
        return vehicle_directly_right;
    }
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::vehiclesRight(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                   const std::shared_ptr<Obstacle> &obstacleK) {

    std::vector<std::shared_ptr<Obstacle>> vehicles_right;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj = vehiclesAdjacent(timeStep, obstacles, obstacleK);

    for (const auto &obs : vehicles_adj) {
        if (obs->leftD(timeStep) < obstacleK->rightD(timeStep)) {
            vehicles_right.push_back(obs);
        }
    }

    return vehicles_right;
}
