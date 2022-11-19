//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <memory>

#include <commonroad_cpp/geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include <commonroad_cpp/predicates/regulatory/has_priority_predicate.h>

bool HasPriorityPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    int prioK{regulatory_elements_utils::getPriority(timeStep, world->getRoadNetwork(), obstacleK,
                                                     additionalFunctionParameters->turningDirection.at(0))};
    int prioP{regulatory_elements_utils::getPriority(timeStep, world->getRoadNetwork(), obstacleP,
                                                     additionalFunctionParameters->turningDirection.at(1))};
    return prioK > prioP and prioK != std::numeric_limits<int>::min() and prioP != std::numeric_limits<int>::min();
}

double HasPriorityPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("HasPriorityPredicate does not support robust evaluation!");
}

Constraint HasPriorityPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("HasPriorityPredicate does not support constraint evaluation!");
}

HasPriorityPredicate::HasPriorityPredicate() : CommonRoadPredicate(true) {}
