//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include <commonroad_cpp/predicates/velocity/slow_leading_vehicle_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

bool SlowLeadingVehiclePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (!inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) or
            !inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            continue;
        double vMax{
            std::min({regulatory_elements_utils::speedLimitSuggested(
                          obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep),
                          TrafficSignTypes::MAX_SPEED),
                      regulatory_elements_utils::typeSpeedLimit(obstacleK->getObstacleType()),
                      parameters.paramMap["roadConditionSpeedLimit"]})};
        if (vMax - obs->getStateByTimeStep(timeStep)->getVelocity() >= parameters.paramMap["minVelocityDif"])
            return true;
    }
    return false;
}

double SlowLeadingVehiclePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("SlowLeadingVehiclePredicate does not support robust evaluation!");
}

Constraint SlowLeadingVehiclePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("SlowLeadingVehiclePredicate does not support constraint evaluation!");
}

SlowLeadingVehiclePredicate::SlowLeadingVehiclePredicate() : CommonRoadPredicate(false) {}
