//
// Created by Sebastian Maierhofer on 07.04.21.
//

#include "in_front_of_predicate.h"

bool InFrontOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::shared_ptr<Obstacle> &obstacleP) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP) > 0;
}

bool InFrontOfPredicate::booleanEvaluation(double lonPositionK, double lonPositionP, double lengthK, double lengthP) {
    return robustEvaluation(lonPositionK, lonPositionP, lengthK, lengthP) > 0;
}

Constraint InFrontOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP) {
    return {obstacleP->frontS(timeStep) + 0.5 * dynamic_cast<Rectangle &>(obstacleK->getGeoShape()).getLength()};
}

Constraint InFrontOfPredicate::constraintEvaluation(double lonPositionP, double lengthK, double lengthP) {
    return {lonPositionP + 0.5 * lengthP + 0.5 * lengthK};
}

double InFrontOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP) {
    return obstacleK->rearS(timeStep) - obstacleP->frontS(timeStep);
}

double InFrontOfPredicate::robustEvaluation(double lonPositionK, double lonPositionP, double lengthK, double lengthP) {
    return (lonPositionK - 0.5 * lengthK) - (lonPositionP + 0.5 * lengthP);
}
