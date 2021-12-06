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
#include "on_access_ramp_predicate.h"

bool OnAccessRampPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {

    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    std::vector<std::shared_ptr<Lanelet>> lanelets = obstacleK->getOccupiedLanelets(timeStep);

    for (auto &l : lanelets){
        std::shared_ptr<Lanelet> lanelet = roadNetwork->findLaneletById(l->getId());
        const std::set<LaneletType> laneletTypes = lanelet->getLaneletTypes();
        for (auto &type : laneletTypes){
            if (type == LaneletType::accessRamp)
                return true;
        }
    }
    return false;
}

double OnAccessRampPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("On Access Ramp Predicate does not support robust evaluation!");
}

Constraint OnAccessRampPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("On Access Ramp Predicate does not support constraint evaluation!");
}