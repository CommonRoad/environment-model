//
// Created by Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "commonroad_cpp/predicates/velocity/in_standstill_predicate.h"
#include "exist_standing_leading_vehicle_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
bool ExistStandingLeadingVehiclePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                             const std::shared_ptr<Obstacle> &obstacleK,
                                                             const std::shared_ptr<Obstacle> &obstacleP) {

    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;
    InStandstillPredicate inStandstillPredicate;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (!inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) or
            !inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            continue;
        if (inStandstillPredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            return true;
    }
    return false;
}

Constraint ExistStandingLeadingVehiclePredicate::constraintEvaluation(size_t timeStep,
                                                                      const std::shared_ptr<World> &world,
                                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                                      const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Exist Standing Leading Vehicle Predicate does not support constraint evaluation!");
}

double ExistStandingLeadingVehiclePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                              const std::shared_ptr<Obstacle> &obstacleK,
                                                              const std::shared_ptr<Obstacle> &obstacleP) {
    throw std::runtime_error("Exist Standing Leading Vehicle Predicate does not support robust evaluation!");
}