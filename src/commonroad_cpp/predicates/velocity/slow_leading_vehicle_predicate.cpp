//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "slow_leading_vehicle_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "keeps_lane_speed_limit_predicate.h"

bool SlowLeadingVehiclePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO consider other speed limit types -> see Python predicate
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;
    KeepsLaneSpeedLimitPredicate keepsLaneSpeedLimitPredicate;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (!inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) or
            !inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            continue;
        double vMaxLane{keepsLaneSpeedLimitPredicate.speedLimitSuggested(
            obstacleK->getOccupiedLanelets(world->getRoadNetwork(), timeStep),
            world->getRoadNetwork()->extractTrafficSignIDForCountry(TrafficSignTypes::MAX_SPEED))};
        if (vMaxLane - obs->getStateByTimeStep(timeStep)->getVelocity() >= parameters.minVelocityDif)
            return true;
    }
    return false;
}

double SlowLeadingVehiclePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add robust mode
    throw std::runtime_error("SlowLeadingVehiclePredicate does not support robust evaluation!");
}

Constraint SlowLeadingVehiclePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                             const std::shared_ptr<Obstacle> &obstacleK,
                                                             const std::shared_ptr<Obstacle> &obstacleP) {
    // TODO add constraint mode
    throw std::runtime_error("SlowLeadingVehiclePredicate does not support constraint evaluation!");
}
