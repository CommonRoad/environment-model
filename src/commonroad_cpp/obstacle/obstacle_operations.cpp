//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "obstacle_operations.h"
#include "../roadNetwork/lanelet//lanelet_operations.h"

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

std::shared_ptr<Obstacle>
obstacle_operations::obstacleDirectlyLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                          const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_left = obstaclesLeft(timeStep, obstacles, obstacleK);
    if (vehicles_left.empty())
        return nullptr;
    else if (vehicles_left.size() == 1)
        return vehicles_left[0];
    else {
        std::shared_ptr<Obstacle> vehicle_directly_left = vehicles_left[0];
        for (const auto &obs : vehicles_left)
            if (obs->getLatPosition(timeStep, obstacleK->getReferenceLane(timeStep)) <
                vehicle_directly_left->getLatPosition(timeStep, obstacleK->getReferenceLane(timeStep)))
                vehicle_directly_left = obs;

        return vehicle_directly_left;
    }
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::obstaclesLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                   const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_left;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj = obstaclesAdjacent(timeStep, obstacles, obstacleK);

    for (const auto &obs : vehicles_adj)
        if (obs->rightD(timeStep, obstacleK->getReferenceLane(timeStep)) > obstacleK->leftD(timeStep))
            vehicles_left.push_back(obs);

    return vehicles_left;
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::obstaclesAdjacent(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                       const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehiclesAdj;
    auto refLaneObstacleK{obstacleK->getReferenceLane(timeStep)};

    for (const auto &obs : obstacles) {
        if (!obs->timeStepExists(timeStep) or obs->getId() == obstacleK->getId())
            continue;
        auto obsRearS{obs->rearS(timeStep, refLaneObstacleK)};
        auto obsFrontS{obs->frontS(timeStep, refLaneObstacleK)};
        if (obsRearS < obstacleK->frontS(timeStep) and obstacleK->frontS(timeStep) < obsFrontS) {
            vehiclesAdj.push_back(obs);
            continue;
        }
        if (obsRearS < obstacleK->rearS(timeStep) and obstacleK->rearS(timeStep) < obsFrontS) {
            vehiclesAdj.push_back(obs);
            continue;
        }
        if (obstacleK->rearS(timeStep) <= obsRearS and obsFrontS <= obstacleK->frontS(timeStep)) {
            vehiclesAdj.push_back(obs);
            continue;
        }
    }

    return vehiclesAdj;
}

std::shared_ptr<Obstacle>
obstacle_operations::obstacleDirectlyRight(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                           const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_right = obstaclesRight(timeStep, obstacles, obstacleK);
    if (vehicles_right.empty())
        return nullptr;
    else if (vehicles_right.size() == 1)
        return vehicles_right[0];
    else {
        std::shared_ptr<Obstacle> vehicle_directly_right = vehicles_right[0];
        for (const auto &obs : vehicles_right)
            if (obs->getLatPosition(timeStep, obstacleK->getReferenceLane(timeStep)) >
                vehicle_directly_right->getLatPosition(timeStep, obstacleK->getReferenceLane(timeStep)))
                vehicle_directly_right = obs;
        return vehicle_directly_right;
    }
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::obstaclesRight(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                    const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_right;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj = obstaclesAdjacent(timeStep, obstacles, obstacleK);

    for (const auto &obs : vehicles_adj)
        if (obs->leftD(timeStep, obstacleK->getReferenceLane(timeStep)) < obstacleK->rightD(timeStep))
            vehicles_right.push_back(obs);

    return vehicles_right;
}

std::set<std::shared_ptr<Lanelet>> obstacle_operations::laneletsRightOfObstacle(size_t timeStep,
                                                                                const std::shared_ptr<Obstacle> &obs) {
    std::set<std::shared_ptr<Lanelet>> rightLanelets;
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obs->getOccupiedLaneletsByShape(timeStep);

    for (auto &occ_l : occupiedLanelets) {
        std::vector<std::shared_ptr<Lanelet>> newLanelets = lanelet_operations::laneletsRightOfLanelet(occ_l);
        for (const auto &lanelet : newLanelets)
            rightLanelets.emplace(lanelet);
    }
    return rightLanelets;
}

std::set<std::shared_ptr<Lanelet>> obstacle_operations::laneletsLeftOfObstacle(size_t timeStep,
                                                                               const std::shared_ptr<Obstacle> &obs) {
    std::set<std::shared_ptr<Lanelet>> leftLanelets;
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obs->getOccupiedLaneletsByShape(timeStep);

    for (auto &occ_l : occupiedLanelets) {
        std::vector<std::shared_ptr<Lanelet>> newLanelets = lanelet_operations::laneletsLeftOfLanelet(occ_l);
        for (const auto &lanelet : newLanelets)
            leftLanelets.emplace(lanelet);
    }
    return leftLanelets;
}