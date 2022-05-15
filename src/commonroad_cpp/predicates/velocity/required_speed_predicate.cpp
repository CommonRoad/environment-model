//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "required_speed_predicate.h"
#include "../../roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

bool RequiredSpeedPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    double vReqLane{regulatory_elements_utils::requiredVelocity(
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep),
        world->getRoadNetwork()->extractTrafficSignIDForCountry(TrafficSignTypes::MIN_SPEED))};
    return vReqLane <= obstacleK->getStateByTimeStep(timeStep)->getVelocity();
}

double RequiredSpeedPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("RequiredSpeedPredicate does not support robust evaluation!");
}

Constraint RequiredSpeedPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("RequiredSpeedPredicate does not support constraint evaluation!");
}
RequiredSpeedPredicate::RequiredSpeedPredicate() : CommonRoadPredicate(false) {}
