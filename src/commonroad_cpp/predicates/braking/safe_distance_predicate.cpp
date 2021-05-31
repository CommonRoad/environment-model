//
// Created by Sebastian Maierhofer on 19.02.21.
//

#include "safe_distance_predicate.h"

double SafeDistancePredicate::computeSafeDistance(double velocityK, double velocityP, double minAccelerationK,
                                                  double minAccelerationP, double tReact) {
    if (minAccelerationK >= 0 or minAccelerationP >= 0)
        throw std::logic_error("Acceleration is not negative!");
    return pow(velocityP, 2) / (-2 * std::abs(minAccelerationP)) -
           pow(velocityK, 2) / (-2 * std::abs(minAccelerationK)) + velocityK * tReact;
}

bool SafeDistancePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP) > 0;
}

bool SafeDistancePredicate::booleanEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP,
                                              double minAccelerationK, double minAccelerationP, double tReact,
                                              double lengthK, double lengthP) {
    return robustEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK, minAccelerationP, tReact, lengthK,
                            lengthP) > 0;
}

Constraint SafeDistancePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP) {
    double aMinK{obstacleK->getAminLong()};
    double aMinP{obstacleP->getAminLong()};
    double tReact{obstacleK->getReactionTime()};
    return {obstacleP->rearS(timeStep) - 0.5 * dynamic_cast<Rectangle &>(obstacleK->getGeoShape()).getLength() -
            computeSafeDistance(obstacleK->getStateByTimeStep(timeStep)->getVelocity(),
                                obstacleP->getStateByTimeStep(timeStep)->getVelocity(), aMinK, aMinP, tReact)};
}

Constraint SafeDistancePredicate::constraintEvaluation(double lonPosP, double velocityK, double velocityP,
                                                       double minAccelerationK, double minAccelerationP, double tReact,
                                                       double lengthK, double lengthP) {
    return {lonPosP - 0.5 * lengthP - 0.5 * lengthK -
            computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact)};
}

double SafeDistancePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP) {
    double aMinK{obstacleK->getAminLong()};
    double aMinP{obstacleP->getAminLong()};
    double tReact{obstacleK->getReactionTime()};
    double dSafe{computeSafeDistance(obstacleK->getStateByTimeStep(timeStep)->getVelocity(),
                                     obstacleP->getStateByTimeStep(timeStep)->getVelocity(), aMinK, aMinP, tReact)};
    double deltaS{obstacleP->rearS(timeStep) - obstacleK->frontS(timeStep)};
    // if pth vehicle is not in front of the kth vehicle, safe distance is not applicable -> return positive robustness
    if (deltaS < 0)
        return abs(deltaS);
    else
        return (deltaS - dSafe);
}

double SafeDistancePredicate::robustEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP,
                                               double minAccelerationK, double minAccelerationP, double tReact,
                                               double lengthK, double lengthP) {
    double dSafe{computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact)};
    double deltaS = (lonPosP - 0.5 * lengthP) - (lonPosK + 0.5 * lengthK);
    // if pth vehicle is not in front of the kth vehicle, safe distance is not applicable -> return positive robustness
    if (deltaS < 0)
        return abs(deltaS);
    else
        return (deltaS - dSafe);
}