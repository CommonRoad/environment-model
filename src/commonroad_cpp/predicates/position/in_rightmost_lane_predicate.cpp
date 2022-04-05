//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/road_network.h"
#include "in_rightmost_lane_predicate.h"

bool InRightmostLanePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    return std::any_of(lanelets.begin(), lanelets.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
        return lanelet->getAdjacentRight().adj == nullptr ||
               lanelet->getAdjacentRight().dir != lanelet->getAdjacentRight().adj->getAdjacentLeft().dir;
    });
}

double InRightmostLanePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> additionalFunctionParameters) {
    throw std::runtime_error("In Rightmost Lane Predicate does not support robust evaluation!");
}

Constraint InRightmostLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> additionalFunctionParameters) {
    throw std::runtime_error("In Rightmost Lane Predicate does not support constraint evaluation!");
}

InRightmostLanePredicate::InRightmostLanePredicate() : CommonRoadPredicate(false) {}
