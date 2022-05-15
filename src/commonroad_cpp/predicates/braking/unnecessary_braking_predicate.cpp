//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/predicates/braking/safe_distance_predicate.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/predicates/general/lane_based_orientation_similar_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"
#include "unnecessary_braking_predicate.h"

bool UnnecessaryBrakingPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return robustEvaluation(timeStep, world, obstacleK) > 0;
}

Constraint UnnecessaryBrakingPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<double> constraintValues;
    InSameLanePredicate sameLanePredicate;
    InFrontOfPredicate inFrontOfPredicate;
    LaneBasedOrientationSimilarPredicate orientationPredicate;
    SafeDistancePredicate safeDistancePredicate;
    if (!obstacleK->getStateByTimeStep(timeStep)->getValidStates().acceleration)
        obstacleK->interpolateAcceleration(timeStep, world->getDt());
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (sameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            orientationPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            safeDistancePredicate.booleanEvaluation(timeStep, world, obstacleK, obs)) {
            if (!obs->getStateByTimeStep(timeStep)->getValidStates().acceleration)
                obs->interpolateAcceleration(timeStep, world->getDt());
            constraintValues.push_back(obs->getStateByTimeStep(timeStep)->getAcceleration() + parameters.aAbrupt);
        }
    }
    if (!constraintValues.empty())
        return {*max_element(constraintValues.begin(), constraintValues.end())};
    else
        return {parameters.aAbrupt};
}

double UnnecessaryBrakingPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::vector<double> robustnessValues;
    InSameLanePredicate sameLanePredicate;
    InFrontOfPredicate inFrontOfPredicate;
    LaneBasedOrientationSimilarPredicate orientationPredicate;
    SafeDistancePredicate safeDistancePredicate;
    if (!obstacleK->getStateByTimeStep(timeStep)->getValidStates().acceleration)
        obstacleK->interpolateAcceleration(timeStep, world->getDt());
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (sameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            inFrontOfPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            orientationPredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            safeDistancePredicate.booleanEvaluation(timeStep, world, obstacleK, obs)) {
            if (!obs->getStateByTimeStep(timeStep)->getValidStates().acceleration)
                obs->interpolateAcceleration(timeStep, world->getDt());
            robustnessValues.push_back(parameters.aAbrupt - obstacleK->getStateByTimeStep(timeStep)->getAcceleration() +
                                       obs->getStateByTimeStep(timeStep)->getAcceleration());
        }
    }
    if (!robustnessValues.empty())
        return *max_element(robustnessValues.begin(), robustnessValues.end());
    else
        return -obstacleK->getStateByTimeStep(timeStep)->getAcceleration() + parameters.aAbrupt;
}

UnnecessaryBrakingPredicate::UnnecessaryBrakingPredicate() : CommonRoadPredicate(false) {}