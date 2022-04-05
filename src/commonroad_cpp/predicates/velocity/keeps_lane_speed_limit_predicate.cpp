//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "keeps_lane_speed_limit_predicate.h"

bool KeepsLaneSpeedLimitPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> additionalFunctionParameters) {
    double vMaxLane{regulatory_elements_utils::speedLimit(
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep),
        world->getRoadNetwork()->extractTrafficSignIDForCountry(TrafficSignTypes::MAX_SPEED))};
    return vMaxLane >= obstacleK->getStateByTimeStep(timeStep)->getVelocity();
}

double KeepsLaneSpeedLimitPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> additionalFunctionParameters) {
    throw std::runtime_error("KeepsLaneSpeedLimitPredicate does not support robust evaluation!");
}

Constraint KeepsLaneSpeedLimitPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> additionalFunctionParameters) {
    throw std::runtime_error("KeepsLaneSpeedLimitPredicate does not support constraint evaluation!");
}
KeepsLaneSpeedLimitPredicate::KeepsLaneSpeedLimitPredicate() : CommonRoadPredicate(false) {}
