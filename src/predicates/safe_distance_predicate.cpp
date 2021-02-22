//
// Created by Sebastian Maierhofer on 19.02.21.
//

#include "safe_distance_predicate.h"

double
SafeDistancePredicate::computeSafeDistance(double velocityK, double velocityP, double minAccelerationK, double minAccelerationP,
                                  double tReact) {
    return pow(velocityP, 2) / (-2 * std::abs(minAccelerationP)) -
           pow(velocityK, 2) / (-2 * std::abs(minAccelerationK)) + velocityK * tReact;
}

bool SafeDistancePredicate::booleanEvaluation(int timeStep,
                                     const std::shared_ptr<RoadNetwork> &roadNetwork,
                                     const std::shared_ptr<Obstacle> &obstacleK,
                                     const std::shared_ptr<Obstacle> &obstacleP,
                                     const std::vector<std::shared_ptr<Obstacle>> &obstacles) {
    return robustEvaluation(timeStep, roadNetwork, obstacleK, obstacleP, obstacles) > 0;
}

bool SafeDistancePredicate::booleanEvaluation(int timeStep,
                                     const std::shared_ptr<RoadNetwork> &roadNetwork,
                                     const std::shared_ptr<Obstacle> &obstacleK,
                                     const std::shared_ptr<Obstacle> &obstacleP) {
    return robustEvaluation(timeStep, roadNetwork, obstacleK, obstacleP) > 0;
}

bool SafeDistancePredicate::booleanEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP,
                                     double minAccelerationK, double minAccelerationP, double tReact) {
    return robustEvaluation(lonPosK, lonPosP, velocityK, velocityP, minAccelerationK, minAccelerationP, tReact) > 0;
}

Constraint SafeDistancePredicate::constraintEvaluation(int timeStep,
                                              const std::shared_ptr<RoadNetwork> &roadNetwork,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::shared_ptr<Obstacle>> &obstacles) {
    return constraintEvaluation(timeStep, roadNetwork, obstacleK, obstacleP);
}

Constraint SafeDistancePredicate::constraintEvaluation(int timeStep,
                                              const std::shared_ptr<RoadNetwork> &roadNetwork,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP) {
    double aMinK {obstacleK->getAminLong() };
    double aMinP {obstacleP->getAminLong() };
    double tReact {obstacleK->getReactionTime() };

    return {computeSafeDistance(obstacleK->getTrajectoryPrediction().at(timeStep)->getVelocity(),
                                obstacleP->getTrajectoryPrediction().at(timeStep)->getVelocity(),
                                aMinK, aMinP, tReact)};
}

Constraint
SafeDistancePredicate::constraintEvaluation(double velocityK, double velocityP, double minAccelerationK, double minAccelerationP,
                                   double tReact) {
    return { computeSafeDistance(velocityK, velocityP, minAccelerationK, minAccelerationP, tReact) };
}

double SafeDistancePredicate::robustEvaluation(int timeStep,
                                      const std::shared_ptr<RoadNetwork> &roadNetwork,
                                      const std::shared_ptr<Obstacle> &obstacleK,
                                      const std::shared_ptr<Obstacle> &obstacleP,
                                      const std::vector<std::shared_ptr<Obstacle>> &obstacles) {
    return robustEvaluation(timeStep, roadNetwork, obstacleK, obstacleP);
}

double SafeDistancePredicate::robustEvaluation(int timeStep,
                                      const std::shared_ptr<RoadNetwork> &roadNetwork,
                                      const std::shared_ptr<Obstacle> &obstacleK,
                                      const std::shared_ptr<Obstacle> &obstacleP) {
    double dSafe { constraintEvaluation(timeStep, roadNetwork, obstacleK, obstacleP).realValuedConstraint };
    double deltaS { obstacleP->rearS(timeStep) - obstacleK->frontS(timeStep) };

    return (deltaS - dSafe);
}

double SafeDistancePredicate::robustEvaluation(double lonPosK, double lonPosP, double velocityK, double velocityP,
                                      double minAccelerationK, double minAccelerationP, double tReact) {
    double dSafe{ constraintEvaluation(velocityK, velocityP, minAccelerationK, minAccelerationP,
                                      tReact).realValuedConstraint };
    double deltaS = lonPosP - lonPosK;

    return (deltaS - dSafe);
}