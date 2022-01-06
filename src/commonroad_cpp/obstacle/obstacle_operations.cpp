//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "obstacle_operations.h"
#include "../geometry/geometric_operations.h"
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
    // use cross product between a line and a point to evaluate whether obstacle is left
    vertex vertA{obstacleK->getOccupancyPolygonShape(timeStep).outer()[1].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[1].y()};
    vertex vertC{obstacleK->getOccupancyPolygonShape(timeStep).outer()[0].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[0].y()};
    vertC.x -= vertA.x;
    vertC.y -= vertA.y;

    for (const auto &obs : vehicles_adj) {
        vertex vertP0{obs->getOccupancyPolygonShape(timeStep).outer()[0].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[0].y()};
        vertex vertP1{obs->getOccupancyPolygonShape(timeStep).outer()[1].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[1].y()};
        vertex vertP2{obs->getOccupancyPolygonShape(timeStep).outer()[2].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[2].y()};
        vertex vertP3{obs->getOccupancyPolygonShape(timeStep).outer()[3].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[3].y()};
        auto crossProductF0{vertC.x * (vertP0.y - vertA.y) - vertC.y * (vertP0.x - vertA.x)};
        auto crossProductF1{vertC.x * (vertP1.y - vertA.y) - vertC.y * (vertP1.x - vertA.x)};
        auto crossProductF2{vertC.x * (vertP2.y - vertA.y) - vertC.y * (vertP2.x - vertA.x)};
        auto crossProductF3{vertC.x * (vertP3.y - vertA.y) - vertC.y * (vertP3.x - vertA.x)};
        if (crossProductF0 < 0 and crossProductF1 < 0 and crossProductF2 < 0 and crossProductF3 < 0)
            vehicles_left.push_back(obs);
    }

    return vehicles_left;
}

std::vector<std::shared_ptr<Obstacle>>
obstacle_operations::obstaclesAdjacent(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                       const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehiclesAdj;
    auto refLaneObstacleK{obstacleK->getReferenceLane(timeStep)};
    // use cross product between a line and a point to evaluate whether obstacle is adjacent
    vertex vertA{obstacleK->getOccupancyPolygonShape(timeStep).outer()[1].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[1].y()};
    vertex vertB{obstacleK->getOccupancyPolygonShape(timeStep).outer()[2].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[2].y()};
    vertex vertC{obstacleK->getOccupancyPolygonShape(timeStep).outer()[0].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[0].y()};
    vertex vertD{obstacleK->getOccupancyPolygonShape(timeStep).outer()[3].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[3].y()};
    vertB.x -= vertA.x;
    vertB.y -= vertA.y;
    vertD.x -= vertC.x;
    vertD.y -= vertC.y;
    for (const auto &obs : obstacles) {
        if (!obs->timeStepExists(timeStep) or obs->getId() == obstacleK->getId())
            continue;
        vertex vertP0{obs->getOccupancyPolygonShape(timeStep).outer()[0].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[0].y()};
        vertex vertP1{obs->getOccupancyPolygonShape(timeStep).outer()[1].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[1].y()};
        vertex vertP2{obs->getOccupancyPolygonShape(timeStep).outer()[2].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[2].y()};
        vertex vertP3{obs->getOccupancyPolygonShape(timeStep).outer()[3].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[3].y()};

        auto crossProductF0{vertB.x * (vertP0.y - vertA.y) - vertB.y * (vertP0.x - vertA.x)};
        auto crossProductF1{vertB.x * (vertP1.y - vertA.y) - vertB.y * (vertP1.x - vertA.x)};
        auto crossProductF2{vertB.x * (vertP2.y - vertA.y) - vertB.y * (vertP2.x - vertA.x)};
        auto crossProductF3{vertB.x * (vertP3.y - vertA.y) - vertB.y * (vertP3.x - vertA.x)};
        auto crossProductH0{vertD.x * (vertP0.y - vertC.y) - vertD.y * (vertP0.x - vertC.x)};
        auto crossProductH1{vertD.x * (vertP1.y - vertC.y) - vertD.y * (vertP1.x - vertC.x)};
        auto crossProductH2{vertD.x * (vertP2.y - vertC.y) - vertD.y * (vertP2.x - vertC.x)};
        auto crossProductH3{vertD.x * (vertP3.y - vertC.y) - vertD.y * (vertP3.x - vertC.x)};

        if (crossProductF0 >= 0 and crossProductF1 >= 0 and crossProductF2 >= 0 and crossProductF3 >= 0)
            continue;
        else if (crossProductH0 <= 0 and crossProductH1 <= 0 and crossProductH2 <= 0 and crossProductH3 <= 0)
            continue;
        else
            vehiclesAdj.push_back(obs);
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
    // use cross product between a line and a point to evaluate whether obstacle is right
    vertex vertA{obstacleK->getOccupancyPolygonShape(timeStep).outer()[2].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[2].y()};
    vertex vertC{obstacleK->getOccupancyPolygonShape(timeStep).outer()[3].x(),
                 obstacleK->getOccupancyPolygonShape(timeStep).outer()[3].y()};
    vertC.x -= vertA.x;
    vertC.y -= vertA.y;

    for (const auto &obs : vehicles_adj) {
        vertex vertP0{obs->getOccupancyPolygonShape(timeStep).outer()[0].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[0].y()};
        vertex vertP1{obs->getOccupancyPolygonShape(timeStep).outer()[1].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[1].y()};
        vertex vertP2{obs->getOccupancyPolygonShape(timeStep).outer()[2].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[2].y()};
        vertex vertP3{obs->getOccupancyPolygonShape(timeStep).outer()[3].x(),
                      obs->getOccupancyPolygonShape(timeStep).outer()[3].y()};
        auto crossProductF0{vertC.x * (vertP0.y - vertA.y) - vertC.y * (vertP0.x - vertA.x)};
        auto crossProductF1{vertC.x * (vertP1.y - vertA.y) - vertC.y * (vertP1.x - vertA.x)};
        auto crossProductF2{vertC.x * (vertP2.y - vertA.y) - vertC.y * (vertP2.x - vertA.x)};
        auto crossProductF3{vertC.x * (vertP3.y - vertA.y) - vertC.y * (vertP3.x - vertA.x)};
        if (crossProductF0 > 0 and crossProductF1 > 0 and crossProductF2 > 0 and crossProductF3 > 0)
            vehicles_right.push_back(obs);
    }
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