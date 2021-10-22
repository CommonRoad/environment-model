//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/in_same_lane_predicate.h>
#include <commonroad_cpp/world.h>

#include "orientation_towards_predicate.h"

bool OrientationTowardsPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP) {
    return (obstacleK->getLatPosition(timeStep) < // right side
                obstacleP->getLatPosition(timeStep, obstacleK->getReferenceLane(timeStep)) and
            obstacleK->getCurvilinearOrientation(timeStep) > 0) or
           (obstacleK->getLatPosition(timeStep) > // left side
                obstacleP->getLatPosition(timeStep, obstacleK->getReferenceLane(timeStep)) and
            obstacleK->getCurvilinearOrientation(timeStep) < 0);
}

double OrientationTowardsPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("CutInPredicate does not support robust evaluation!");
}

Constraint OrientationTowardsPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                             const std::shared_ptr<Obstacle> &obstacleK,
                                                             const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("CutInPredicate does not support constraint evaluation!");
}
