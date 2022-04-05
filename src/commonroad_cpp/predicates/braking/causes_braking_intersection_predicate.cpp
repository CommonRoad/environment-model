//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/predicates/braking/safe_distance_predicate.h>
#include <commonroad_cpp/world.h>

#include "causes_braking_intersection_predicate.h"

bool CausesBrakingIntersectionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           OptionalPredicateParameters additionalFunctionParameters) {
    if (!obstacleK->getStateByTimeStep(timeStep)->getValidStates().acceleration)
        obstacleK->interpolateAcceleration(timeStep, world->getDt());
    if (!obstacleP->getStateByTimeStep(timeStep)->getValidStates().acceleration)
        obstacleP->interpolateAcceleration(timeStep, world->getDt());

    auto distance{obstacleK->rearS(timeStep, obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)) -
                  obstacleP->frontS(world->getRoadNetwork(), timeStep)};
    return 0 <= distance and distance <= parameters.dBrakingIntersection and
           static_cast<double>(obstacleP->getStateByTimeStep(timeStep)->getValidStates().acceleration) <
               parameters.aBrakingIntersection;
}

Constraint CausesBrakingIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

double CausesBrakingIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                            const std::shared_ptr<Obstacle> &obstacleK,
                                                            const std::shared_ptr<Obstacle> &obstacleP,
                                                            OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

CausesBrakingIntersectionPredicate::CausesBrakingIntersectionPredicate() : CommonRoadPredicate(false) {}