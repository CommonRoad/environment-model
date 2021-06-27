//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "in_standstill_predicate.h"

bool InStandstillPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP) {
    return (-parameters.standstillError < obstacleK->getStateByTimeStep(timeStep)->getVelocity() and
            parameters.standstillError > obstacleK->getStateByTimeStep(timeStep)->getVelocity());
}

double InStandstillPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add robust mode
    throw std::runtime_error("InStandstillPredicate does not support robust evaluation!");
}

Constraint InStandstillPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add constraint mode
    throw std::runtime_error("InStandstillPredicate does not support constraint evaluation!");
}
