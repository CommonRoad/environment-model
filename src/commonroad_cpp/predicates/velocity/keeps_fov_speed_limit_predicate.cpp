//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/keeps_fov_speed_limit_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

bool KeepsFOVSpeedLimitPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return obstacleK->getStateByTimeStep(timeStep)->getVelocity() <= parameters.fovSpeedLimit;
}

double KeepsFOVSpeedLimitPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("KeepsTypeSpeedLimitPredicate does not support robust evaluation!");
}

Constraint KeepsFOVSpeedLimitPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("KeepsFOVSpeedLimitPredicate does not support constraint evaluation!");
}

KeepsFOVSpeedLimitPredicate::KeepsFOVSpeedLimitPredicate() : CommonRoadPredicate(false) {}
