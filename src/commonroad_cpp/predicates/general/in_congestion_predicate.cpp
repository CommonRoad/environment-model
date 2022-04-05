//
// Created by Sebastian Maierhofer and Evald Nexhipi.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//
#include "in_congestion_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
bool InCongestionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              OptionalPredicateParameters additionalFunctionParameters) {
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;

    size_t num_vehicles = 0;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (obs->getStateByTimeStep(timeStep)->getVelocity() <= parameters.maxCongestionVelocity and
            inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            num_vehicles += 1;
    }
    return num_vehicles >= parameters.numVehCongestion;
}

Constraint InCongestionPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("In Congestion Predicate does not support constraint evaluation!");
}

double InCongestionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               OptionalPredicateParameters additionalFunctionParameters) {
    throw std::runtime_error("In Congestion Predicate does not support robust evaluation!");
}

InCongestionPredicate::InCongestionPredicate() : CommonRoadPredicate(false) {}
