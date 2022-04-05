//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "keeps_type_speed_limit_predicate.h"
#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/obstacle/obstacle.h>

bool KeepsTypeSpeedLimitPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     OptionalPredicateParameters additionalFunctionParameters) {
    return obstacleK->getStateByTimeStep(timeStep)->getVelocity() <=
           regulatory_elements_utils::typeSpeedLimit(obstacleK->getObstacleType());
}

double KeepsTypeSpeedLimitPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("KeepsTypeSpeedLimitPredicate does not support robust evaluation!");
}

Constraint KeepsTypeSpeedLimitPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("KeepsTypeSpeedLimitPredicate does not support constraint evaluation!");
}

KeepsTypeSpeedLimitPredicate::KeepsTypeSpeedLimitPredicate() : CommonRoadPredicate(false) {}
