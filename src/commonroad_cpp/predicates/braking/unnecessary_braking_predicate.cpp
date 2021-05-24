//
// Created by Sebastian Maierhofer on 17.05.21.
//

#include "unnecessary_braking_predicate.h"
#include "commonroad_cpp/predicates/braking/safe_distance_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/position/in_same_lane_predicate.h"

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
        if (inFrontOfPredicate.booleanEvaluation(timeStep, world, obs, obstacleK) and
            inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            safeDistancePredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            constraintValues.push_back(obs->getStateByTimeStep(timeStep)->getAcceleration() +
                                       -2); // TODO replace -2 with a_abrupt parameter
    }
    if (constraintValues.size())
        return {*max_element(constraintValues.begin(), constraintValues.end())};
    else
        return {-2}; // TODO replace -2 with a_abrupt parameter
}

double UnnecessaryBrakingPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP) {
    std::vector<double> robustnessValues;
    InFrontOfPredicate inFrontOfPredicate;
    InSameLanePredicate inSameLanePredicate;
    SafeDistancePredicate safeDistancePredicate;
    for (const auto &obs : world->getObstacles()) {
        if (!obs->timeStepExists(timeStep))
            continue;
        if (inFrontOfPredicate.booleanEvaluation(timeStep, world, obs, obstacleK) and
            inSameLanePredicate.booleanEvaluation(timeStep, world, obstacleK, obs) and
            safeDistancePredicate.booleanEvaluation(timeStep, world, obstacleK, obs))
            robustnessValues.push_back(
                -2 - obstacleK->getStateByTimeStep(timeStep)->getAcceleration() +
                obs->getStateByTimeStep(timeStep)->getAcceleration()); // TODO replace -2 with a_abrupt parameter
    }
    if (robustnessValues.size())
        return *max_element(robustnessValues.begin(), robustnessValues.end());
    else
        return -obstacleK->getStateByTimeStep(timeStep)->getAcceleration() +
               -2; // TODO replace -2 with a_abrupt parameter
}