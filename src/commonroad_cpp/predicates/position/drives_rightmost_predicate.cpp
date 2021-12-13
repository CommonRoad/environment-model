//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/road_network.h"
#include "drives_rightmost_predicate.h"

bool DrivesRightmostPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {

    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacleK->getOccupiedLanelets(timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_right = vehicleDirectlyRight(timeStep, world, obstacleK);

    if (vehicle_directly_right != nullptr) {
        if ((obstacleK->rightD(timeStep) - vehicle_directly_right->leftD(timeStep)) < parameters.closeToOtherVehicle)
            return true;
        else
            return false;
    } else {
        double right_position = obstacleK->rightD(timeStep);
        double s_ego = obstacleK->getLonPosition(timeStep);
        std::vector<std::shared_ptr<Lane>> lanes;
        for (auto &occLa : occupiedLanelets) {
            std::vector<std::shared_ptr<Lane>> lanesOfLanelet =
                roadNetwork->findLanesByContainedLanelet(occLa->getId());
            for (auto &la : lanesOfLanelet) {
                lanes.push_back(la);
            }
        }

        for (auto &lane : lanes) {
            if (0.5 * lane->getWidth(s_ego) + right_position > parameters.closeToLaneBorder)
                return false;
        }
        return true;
    }
}

double DrivesRightmostPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives Rightmost Predicate does not support robust evaluation!");
}

Constraint DrivesRightmostPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Drives Rightmost Predicate does not support constraint evaluation!");
}

std::shared_ptr<Obstacle> DrivesRightmostPredicate::vehicleDirectlyRight(size_t timeStep,
                                                                         const std::shared_ptr<World> &world,
                                                                         const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_right = vehiclesRight(timeStep, world, obstacleK);
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
DrivesRightmostPredicate::vehiclesRight(size_t timeStep, const std::shared_ptr<World> &world,
                                        const std::shared_ptr<Obstacle> &obstacleK) {

    std::vector<std::shared_ptr<Obstacle>> vehicles_right;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj = vehiclesAdjacent(timeStep, world, obstacleK);

    for (const auto &obs : vehicles_adj) {
        if (obs->leftD(timeStep) < obstacleK->rightD(timeStep)) {
            vehicles_right.push_back(obs);
        }
    }

    return vehicles_right;
}

std::vector<std::shared_ptr<Obstacle>>
DrivesRightmostPredicate::vehiclesAdjacent(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehiclesAdj;
    std::vector<std::shared_ptr<Obstacle>> otherVehicles;

    for (const auto &obs : world->getObstacles()) {
        if (obs != obstacleK) {
            otherVehicles.push_back(obs);
        }
    }

    for (const auto &obs : otherVehicles) {
        if (!obs->getLonPosition(timeStep))
            continue;
        if (obs->rearS(timeStep) < obstacleK->frontS(timeStep) < obs->frontS(timeStep)) {
            vehiclesAdj.push_back(obs);
            continue;
        }
        if (obs->rearS(timeStep) < obstacleK->rearS(timeStep) < obs->frontS(timeStep)) {
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
DrivesRightmostPredicate::DrivesRightmostPredicate(): CommonRoadPredicate(false) {}