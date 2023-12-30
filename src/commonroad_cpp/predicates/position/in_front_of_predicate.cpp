//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/in_front_of_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool InFrontOfPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return robustEvaluation(timeStep, world, obstacleP, obstacleK) > 0;
}

bool InFrontOfPredicate::booleanEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK) {
    return robustEvaluation(lonPositionP, lonPositionK, lengthP, lengthK) > 0;
}

Constraint InFrontOfPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return {obstacleP->frontS(world->getRoadNetwork(), timeStep) +
            0.5 * dynamic_cast<Rectangle &>(obstacleK->getGeoShape()).getLength()};
}

Constraint InFrontOfPredicate::constraintEvaluation(double lonPositionP, double lengthK, double lengthP) {
    return {lonPositionP + 0.5 * lengthP + 0.5 * lengthK};
}

double
InFrontOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                     const std::shared_ptr<Obstacle> &obstacleP,
                                     const std::shared_ptr<Obstacle> &obstacleK,
                                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    // check whether kth obstacle is in the projection domain of the reference of the pth vehicle -> otherwise ccs
    // fails
    if (!obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)
             ->getCurvilinearCoordinateSystem()
             ->cartesianPointInProjectionDomain(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                                obstacleK->getStateByTimeStep(timeStep)->getYPosition()))
        return false; // todo logging
    return obstacleK->rearS(timeStep, obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)) -
           obstacleP->frontS(world->getRoadNetwork(), timeStep);
}

double InFrontOfPredicate::robustEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK) {
    return (lonPositionK - 0.5 * lengthK) - (lonPositionP + 0.5 * lengthP);
}

InFrontOfPredicate::InFrontOfPredicate() : CommonRoadPredicate(true) {}
