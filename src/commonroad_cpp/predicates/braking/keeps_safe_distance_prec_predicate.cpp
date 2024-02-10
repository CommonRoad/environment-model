//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <cmath>
#include <stdexcept>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

double KeepsSafeDistancePrecPredicate::computeSafeDistance(double velocityK, double velocityP, double minAccelerationK,
                                                           double minAccelerationP, double tReact) {
    if (minAccelerationK >= 0 or minAccelerationP >= 0)
        throw std::logic_error("Acceleration is not negative!");
    return pow(velocityP, 2) / (-2 * std::abs(minAccelerationP)) -
           pow(velocityK, 2) / (-2 * std::abs(minAccelerationK)) + velocityK * tReact;
}

bool KeepsSafeDistancePrecPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters) > 0;
}

bool KeepsSafeDistancePrecPredicate::booleanEvaluation(double lonPosK, double lonPosP, double velocityK,
                                                       double velocityP, double minAccelerationK,
                                                       double minAccelerationP, double tReact, double lengthK,
                                                       double lengthP) {
    return robustEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK, minAccelerationP, tReact, lengthK,
                            lengthP) > 0;
}

Constraint KeepsSafeDistancePrecPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    double aMinK{obstacleK->getAminLong()};
    double aMinP{obstacleP->getAminLong()};
    double tReact;
    try {
        tReact = obstacleK->getReactionTime().value();
    } catch (const std::bad_optional_access &e) {
        throw std::logic_error{"tried to evaluate KeepsSafeDistancePrecPredicate on an obstacle which does not have a "
                               "reaction time defined"};
    }

    return {obstacleP->rearS(timeStep, obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)) -
            0.5 * dynamic_cast<Rectangle &>(obstacleK->getGeoShape()).getLength() -
            computeSafeDistance(obstacleK->getStateByTimeStep(timeStep)->getVelocity(),
                                obstacleP->getStateByTimeStep(timeStep)->getVelocity(), aMinK, aMinP, tReact)};
}

Constraint KeepsSafeDistancePrecPredicate::constraintEvaluation(double lonPosP, double velocityK, double velocityP,
                                                                double minAccelerationK, double minAccelerationP,
                                                                double tReact, double lengthK, double lengthP) {
    return {lonPosP - 0.5 * lengthP - 0.5 * lengthK -
            computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact)};
}

double KeepsSafeDistancePrecPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    double aMinK{obstacleK->getAminLong()};
    double aMinP{obstacleP->getAminLong()};
    double tReact;
    try {
        tReact = obstacleK->getReactionTime().value();
    } catch (const std::bad_optional_access &e) {
        throw std::logic_error{"tried to evaluate KeepsSafeDistancePrecPredicate on an obstacle which does not have a "
                               "reaction time defined"};
    }
    double dSafe{computeSafeDistance(obstacleK->getStateByTimeStep(timeStep)->getVelocity(),
                                     obstacleP->getStateByTimeStep(timeStep)->getVelocity(), aMinK, aMinP, tReact)};
    double deltaS{0};
    // check whether pth vehicle is in projection domain, e.g. for intersections
    if (obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)
            ->getCurvilinearCoordinateSystem()
            ->cartesianPointInProjectionDomain(obstacleP->getStateByTimeStep(timeStep)->getXPosition(),
                                               obstacleP->getStateByTimeStep(timeStep)->getYPosition()))
        deltaS = obstacleP->rearS(timeStep, obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)) -
                 obstacleK->frontS(world->getRoadNetwork(), timeStep);
    else
        return parameters.getParam("epsilon");
    // if pth vehicle is not in front of the kth vehicle, safe distance is not applicable -> return positive
    // robustness
    if (deltaS < 0)
        return std::abs(deltaS);
    else if (additionalFunctionParameters != nullptr && (deltaS - additionalFunctionParameters->minSafetyDistance) < 0)
        return std::min(deltaS - additionalFunctionParameters->minSafetyDistance, deltaS - dSafe);
    else
        return (deltaS - dSafe);
}

double KeepsSafeDistancePrecPredicate::robustEvaluation(double lonPosK, double lonPosP, double velocityK,
                                                        double velocityP, double minAccelerationK,
                                                        double minAccelerationP, double tReact, double lengthK,
                                                        double lengthP) {
    double dSafe{computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact)};
    double deltaS = (lonPosP - 0.5 * lengthP) - (lonPosK + 0.5 * lengthK);
    // if pth vehicle is not in front of the kth vehicle, safe distance is not applicable -> return positive robustness
    if (deltaS < 0)
        return std::abs(deltaS);
    else
        return (deltaS - dSafe);
}

KeepsSafeDistancePrecPredicate::KeepsSafeDistancePrecPredicate() : CommonRoadPredicate(true) {}