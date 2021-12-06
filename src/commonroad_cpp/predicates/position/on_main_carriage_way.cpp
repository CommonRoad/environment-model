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
#include "on_main_carriage_way.h"

bool OnMainCarriageWayPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,const std::shared_ptr<Obstacle> &obstacleP) {

    std::vector<std::shared_ptr<Lanelet>> lanelets = obstacleK->getOccupiedLanelets(timeStep);
    const std::shared_ptr<RoadNetwork> roadNetwork = world->getRoadNetwork();
    for (const auto &l : lanelets){
        std::shared_ptr<Lanelet> lanelet = roadNetwork->findLaneletById(l->getId());
        std::set<LaneletType> laneletTypes = lanelet->getLaneletTypes();
        for (LaneletType lt : laneletTypes){
            if (lt == LaneletType::mainCarriageWay)
                return true;
        }
    }
    return false;
}

double OnMainCarriageWayPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("On Main Carriage Way Predicate does not support robust evaluation!");
}

Constraint OnMainCarriageWayPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("On Main Carriage Way Predicate does not support constraint evaluation!");
}
