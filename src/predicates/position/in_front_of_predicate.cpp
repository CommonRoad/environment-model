//
// Created by Sebastian Maierhofer on 07.04.21.
//

#include "in_front_of_predicate.h"

bool InFrontOfPredicate::booleanEvaluation(size_t timeStep,
                                  const std::shared_ptr<World> &world,
                                  const std::shared_ptr<Obstacle> &obstacleK,
                                  const std::shared_ptr<Obstacle> &obstacleP) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP) > 0;
}

bool InFrontOfPredicate::booleanEvaluation(double lonPositionK, double lonPositionP) {
    return robustEvaluation(lonPositionK, lonPositionP) > 0;
}

Constraint InFrontOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP) {
    return { obstacleK->frontS(timeStep) };
}

Constraint InFrontOfPredicate::constraintEvaluation(double lonPositionK, double lonPositionP) {
    return { lonPositionK };
}

double InFrontOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP) {
    return obstacleK->frontS(timeStep) - obstacleP->frontS(timeStep);
}

double InFrontOfPredicate::robustEvaluation(double lonPositionK, double lonPositionP) {
    return lonPositionK - lonPositionP;
}


