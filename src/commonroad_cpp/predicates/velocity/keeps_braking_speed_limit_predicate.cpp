//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/keeps_braking_speed_limit_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

bool KeepsBrakingSpeedLimitPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return obstacleK->getStateByTimeStep(timeStep)->getVelocity() <= parameters.getParam("brakingSpeedLimit");
}

double KeepsBrakingSpeedLimitPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("KeepsBrakingSpeedLimitPredicate does not support robust evaluation!");
}

Constraint KeepsBrakingSpeedLimitPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("KeepsBrakingSpeedLimitPredicate does not support constraint evaluation!");
}

KeepsBrakingSpeedLimitPredicate::KeepsBrakingSpeedLimitPredicate() : CommonRoadPredicate(false) {}
