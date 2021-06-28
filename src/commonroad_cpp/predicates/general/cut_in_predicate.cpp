//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/in_same_lane_predicate.h>
#include <commonroad_cpp/world.h>

#include "cut_in_predicate.h"

bool CutInPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                       const std::shared_ptr<Obstacle> &obstacleK,
                                       const std::shared_ptr<Obstacle> &obstacleP) {
    if (obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep).size() == 1)
        return false;

    InSameLanePredicate inSameLanePredicate;
    if (!inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP))
        return false;
    return (obstacleK->getLatPosition(timeStep) < obstacleP->getLatPosition(timeStep) and
            obstacleK->getCurvilinearOrientation(timeStep) > 0) or
           (obstacleK->getLatPosition(timeStep) > obstacleP->getLatPosition(timeStep) and
            obstacleK->getCurvilinearOrientation(timeStep) < 0);
}

double CutInPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                        const std::shared_ptr<Obstacle> &obstacleK,
                                        const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("CutInPredicate does not support robust evaluation!");
}

Constraint CutInPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("CutInPredicate does not support constraint evaluation!");
}
