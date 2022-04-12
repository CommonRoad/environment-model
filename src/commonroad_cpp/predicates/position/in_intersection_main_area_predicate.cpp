//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>
#include <map>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

#include "in_intersection_main_area_predicate.h"

bool InIntersectionMainAreaPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto lanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    for (const auto &lanelet : lanelets) {
        if (std::any_of(lanelet->getLaneletTypes().begin(), lanelet->getLaneletTypes().end(),
                        [](LaneletType laType) { return laType == LaneletType::intersection; }))
            return true;
    }
    return false;
}

double InIntersectionMainAreaPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InIntersectionMainAreaPredicate does not support robust evaluation!");
}

Constraint InIntersectionMainAreaPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InIntersectionMainAreaPredicate does not support constraint evaluation!");
}

InIntersectionMainAreaPredicate::InIntersectionMainAreaPredicate() : CommonRoadPredicate(false) {}
