//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "in_same_lane_predicate.h"

bool InSameLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP) {
    for (const auto &laneK : obstacleK->getDrivingPathLanes(world->getRoadNetwork(), timeStep)) {
        auto relevantIDs{laneK->getContainedLaneletIDs()};
        for (const auto &laneletP : obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep))
            if (relevantIDs.find(laneletP->getId()) != relevantIDs.end())
                return true;
    }
    return false;
}

double InSameLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("In Same Lane Predicate does not support robust evaluation!");
}

Constraint InSameLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("In Same Lane Predicate does not support constraint evaluation!");
}
InSameLanePredicate::InSameLanePredicate() : CommonRoadPredicate(true) {}
