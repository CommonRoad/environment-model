//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "brakes_stronger_predicate.h"
#include <algorithm>
#include <commonroad_cpp/obstacle/obstacle.h>

bool BrakesStrongerPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP) > 0;
}

Constraint BrakesStrongerPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP) {
    return {std::min(obstacleP->getStateByTimeStep(timeStep)->getAcceleration(), 0.0)};
}

double BrakesStrongerPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP) {
    return std::min(obstacleP->getStateByTimeStep(timeStep)->getAcceleration(), 0.0) -
           obstacleK->getStateByTimeStep(timeStep)->getAcceleration();
}

BrakesStrongerPredicate::BrakesStrongerPredicate() : CommonRoadPredicate(false) {}