//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "in_same_lane_predicate.h"

bool InSameLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP) {
    for (const auto &laneP : obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep)) {
        for (const auto &laneK : obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep)) {
            if (laneP->getId() == laneK->getId()) {
                return true;
            }
        }
    }
    return false;
}

double InSameLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("InSameLanePredicate does not support robust evaluation!");
}

Constraint InSameLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("InSameLanePredicate does not support constraint evaluation!");
}
