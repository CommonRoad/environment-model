//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>

#include "in_front_of_predicate.h"

bool InFrontOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleP,
                                           const std::shared_ptr<Obstacle> &obstacleK) {
    return robustEvaluation(timeStep, world, obstacleP, obstacleK) > 0;
}

bool InFrontOfPredicate::booleanEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK) {
    return robustEvaluation(lonPositionP, lonPositionK, lengthP, lengthK) > 0;
}

Constraint InFrontOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::shared_ptr<Obstacle> &obstacleK) {
    return {obstacleP->frontS(timeStep) + 0.5 * dynamic_cast<Rectangle &>(obstacleK->getGeoShape()).getLength()};
}

Constraint InFrontOfPredicate::constraintEvaluation(double lonPositionP, double lengthK, double lengthP) {
    return {lonPositionP + 0.5 * lengthP + 0.5 * lengthK};
}

double InFrontOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::shared_ptr<Obstacle> &obstacleK) {
    try {
        return obstacleK->rearS(timeStep, obstacleP->getReferenceLane(timeStep)) - obstacleP->frontS(timeStep);
    } catch (std::runtime_error) { // not correct, quick-fix for bug in ccs
        auto pathLengthK{obstacleP->getReferenceLane(timeStep)->getPathLength().at(
            obstacleP->getReferenceLane(timeStep)->findClosestIndex(
                obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                obstacleK->getStateByTimeStep(timeStep)->getYPosition()))};
        auto pathLengthP{obstacleP->getReferenceLane(timeStep)->getPathLength().at(
            obstacleP->getReferenceLane(timeStep)->findClosestIndex(
                obstacleP->getStateByTimeStep(timeStep)->getXPosition(),
                obstacleP->getStateByTimeStep(timeStep)->getYPosition()))};
        return (pathLengthK - obstacleK->getGeoShape().getLength() / 2) -
               (pathLengthP + obstacleP->getGeoShape().getLength() / 2);
    }
}

double InFrontOfPredicate::robustEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK) {
    return (lonPositionK - 0.5 * lengthK) - (lonPositionP + 0.5 * lengthP);
}
