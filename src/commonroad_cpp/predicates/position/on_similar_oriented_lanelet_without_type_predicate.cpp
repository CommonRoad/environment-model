//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/geometry/geometric_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/road_network.h"
#include "on_similar_oriented_lanelet_without_type_predicate.h"

bool OnSimilarOrientedLaneletWithoutTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto lanelets{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep)};
    return std::all_of(lanelets.begin(), lanelets.end(),
                       [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                           return !lanelet->hasLaneletType(additionalFunctionParameters->laneletType.at(0));
                       });
}

double OnSimilarOrientedLaneletWithoutTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithoutTypePredicate does not support robust evaluation!");
}

Constraint OnSimilarOrientedLaneletWithoutTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithoutTypePredicate does not support constraint evaluation!");
}
OnSimilarOrientedLaneletWithoutTypePredicate::OnSimilarOrientedLaneletWithoutTypePredicate()
    : CommonRoadPredicate(false) {}
