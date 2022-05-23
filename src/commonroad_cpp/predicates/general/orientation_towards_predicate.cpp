//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/in_same_lane_predicate.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/general/orientation_towards_predicate.h>

bool OrientationTowardsPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto referenceLaneK{obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
    return (obstacleK->getLatPosition(world->getRoadNetwork(), timeStep) < // k on right side
                obstacleP->getLatPosition(timeStep, referenceLaneK) and
            obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep) > 0) or
           (obstacleK->getLatPosition(world->getRoadNetwork(), timeStep) > // k on left side
                obstacleP->getLatPosition(timeStep, referenceLaneK) and
            obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep) < 0);
}

double OrientationTowardsPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OrientationTowardsPredicate does not support robust evaluation!");
}

Constraint OrientationTowardsPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OrientationTowardsPredicate does not support constraint evaluation!");
}
OrientationTowardsPredicate::OrientationTowardsPredicate() : CommonRoadPredicate(true) {}
