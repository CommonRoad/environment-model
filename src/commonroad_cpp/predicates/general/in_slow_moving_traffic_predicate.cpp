//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "in_slow_moving_traffic_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
bool InSlowMovingTrafficPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;

    size_t num_vehicles = 0;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (obs->getStateByTimeStep(timeStep)->getVelocity() <= parameters.maxSlowMovingTrafficVelocity and
            inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            num_vehicles += 1;
    }
    return num_vehicles >= parameters.numVehSlowMovingTraffic;
}

Constraint InSlowMovingTrafficPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Slow Moving Traffic Predicate does not support constraint evaluation!");
}

double InSlowMovingTrafficPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Slow Moving Traffic Predicate does not support robust evaluation!");
}
InSlowMovingTrafficPredicate::InSlowMovingTrafficPredicate() : CommonRoadPredicate(false) {}
