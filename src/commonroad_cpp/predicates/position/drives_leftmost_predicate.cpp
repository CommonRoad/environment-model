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
#include "drives_leftmost_predicate.h"

bool DrivesLeftmostPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {

    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets = obstacleK->getOccupiedLanelets(timeStep);
    std::shared_ptr<Obstacle> vehicle_directly_left = vehicleDirectlyLeft(timeStep, world, obstacleK);

    if (vehicle_directly_left != nullptr) {
        // right_d left_d
        if ((vehicle_directly_left->rightD(timeStep) - obstacleK->leftD(timeStep)) < parameters.closeToOtherVehicle)
            return true;
        else
            return false;
    } else {
        double left_position = obstacleK->leftD(timeStep);
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
            if (0.5 * lane->getWidth(s_ego) - left_position > parameters.closeToLaneBorder)
                return false;
        }
        return true;
    }
}

double DrivesLeftmostPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("DrivesLeftmostPredicate does not support robust evaluation!");
}

Constraint DrivesLeftmostPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("DrivesLeftmostPredicate does not support constraint evaluation!");
}

std::shared_ptr<Obstacle> DrivesLeftmostPredicate::vehicleDirectlyLeft(size_t timeStep,
                                                                       const std::shared_ptr<World> &world,
                                                                       const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehicles_left = vehiclesLeft(timeStep, world, obstacleK);
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
DrivesLeftmostPredicate::vehiclesLeft(size_t timeStep, const std::shared_ptr<World> &world,
                                      const std::shared_ptr<Obstacle> &obstacleK) {

    std::vector<std::shared_ptr<Obstacle>> vehicles_left;
    std::vector<std::shared_ptr<Obstacle>> vehicles_adj = vehiclesAdjacent(timeStep, world, obstacleK);

    for (const auto &obs : vehicles_adj) {
        // right_d , left_d
        if (obs->rightD(timeStep) > obstacleK->leftD(timeStep)) {
            vehicles_left.push_back(obs);
        }
    }

    return vehicles_left;
}

std::vector<std::shared_ptr<Obstacle>>
DrivesLeftmostPredicate::vehiclesAdjacent(size_t timeStep, const std::shared_ptr<World> &world,
                                          const std::shared_ptr<Obstacle> &obstacleK) {
    std::vector<std::shared_ptr<Obstacle>> vehiclesAdj;
    std::vector<std::shared_ptr<Obstacle>> otherVehicles;

    for (const auto &obs : world->getObstacles()) {
        if (obs != obstacleK) {
            otherVehicles.push_back(obs);
        }
    }

    for (const auto &obs : otherVehicles) {
        // also check if obs != obstacleK
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
