//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <memory>

#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/world.h>

#include "has_priority_predicate.h"
#include "same_priority_predicate.h"

bool SamePriorityPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    HasPriorityPredicate hasPriorityPredicate;
    return !hasPriorityPredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP,
                                                   additionalFunctionParameters) and
           !hasPriorityPredicate.booleanEvaluation(timeStep, world, obstacleP, obstacleK, additionalFunctionParameters);
}

double SamePriorityPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("SamePriorityPredicate does not support robust evaluation!");
}

Constraint SamePriorityPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("SamePriorityPredicate does not support constraint evaluation!");
}

SamePriorityPredicate::SamePriorityPredicate() : CommonRoadPredicate(true) {}
