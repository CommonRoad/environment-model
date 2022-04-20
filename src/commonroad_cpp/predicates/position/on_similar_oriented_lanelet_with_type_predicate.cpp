//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/road_network.h"
#include "on_similar_oriented_lanelet_with_type_predicate.h"

bool OnSimilarOrientedLaneletWithTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto lanelets{obstacle_operations::getSimilarlyOrientedLanelets(
        world->getRoadNetwork(), obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep),
        obstacleK->getStateByTimeStep(timeStep), parameters.laneletOccupancySimilarity)};
    return std::any_of(lanelets.begin(), lanelets.end(),
                       [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                           return lanelet->hasLaneletType(additionalFunctionParameters->laneletType.at(0));
                       });
}

double OnSimilarOrientedLaneletWithTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithTypePredicate does not support robust evaluation!");
}

Constraint OnSimilarOrientedLaneletWithTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithTypePredicate does not support constraint evaluation!");
}
OnSimilarOrientedLaneletWithTypePredicate::OnSimilarOrientedLaneletWithTypePredicate() : CommonRoadPredicate(false) {}
