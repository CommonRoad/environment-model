//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/obstacle/obstacle.h"
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/predicates/general/lane_based_orientation_similar_predicate.h>
#include <commonroad_cpp/world.h>
#include <stdexcept>

bool LaneBasedOrientationSimilarPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return std::abs(geometric_operations::subtractOrientations(
               obstacleK->getCurvilinearOrientation(timeStep,
                                                    obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)),
               obstacleP->getCurvilinearOrientation(world->getRoadNetwork(), timeStep))) <
           parameters.getParam("laneMatchingOrientation");
}

Constraint LaneBasedOrientationSimilarPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Lane Based Orientation Similar does not support constraint evaluation!");
}

double LaneBasedOrientationSimilarPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Lane Based Orientation Similar does not support robust evaluation!");
}
LaneBasedOrientationSimilarPredicate::LaneBasedOrientationSimilarPredicate() : CommonRoadPredicate(true) {}
