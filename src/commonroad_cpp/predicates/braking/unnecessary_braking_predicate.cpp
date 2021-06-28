//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>
#include <memory>
#include <vector>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/predicates/braking/safe_distance_predicate.h>
#include <commonroad_cpp/predicates/position/in_front_of_predicate.h>
#include <commonroad_cpp/predicates/position/in_same_lane_predicate.h>
#include <commonroad_cpp/world.h>

#include "unnecessary_braking_predicate.h"

bool UnnecessaryBrakingPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP) {
    return robustEvaluation(timeStep, world, obstacleK) > 0;
}

Constraint UnnecessaryBrakingPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                             const std::shared_ptr<Obstacle> &obstacleK,
                                                             const std::shared_ptr<Obstacle> &obstacleP) {
    std::vector<double> constraintValues;
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;
    SafeDistancePredicate safeDistancePredicate;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            safeDistancePredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            constraintValues.push_back(obs->getStateByTimeStep(timeStep)->getAcceleration() + parameters.aAbrupt);
    }
    if (constraintValues.size())
        return {*max_element(constraintValues.begin(), constraintValues.end())};
    else
        return {parameters.aAbrupt};
}

double UnnecessaryBrakingPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    std::vector<double> robustnessValues;
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;
    SafeDistancePredicate safeDistancePredicate;
    if (!obstacleK->getStateByTimeStep(timeStep)->getValidStates().acceleration)
        obstacleK->interpolateAcceleration(timeStep);
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            safeDistancePredicate.booleanEvaluation(timeStep, world, obstacleK, obs)) {
            if (!obs->getStateByTimeStep(timeStep)->getValidStates().acceleration)
                obs->interpolateAcceleration(timeStep);
            robustnessValues.push_back(parameters.aAbrupt - obstacleK->getStateByTimeStep(timeStep)->getAcceleration() +
                                       obs->getStateByTimeStep(timeStep)->getAcceleration());
        }
    }
    if (robustnessValues.size())
        return *max_element(robustnessValues.begin(), robustnessValues.end());
    else
        return -obstacleK->getStateByTimeStep(timeStep)->getAcceleration() + parameters.aAbrupt;
}