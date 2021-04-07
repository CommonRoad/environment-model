//
// Created by Sebastian Maierhofer on 19.02.21.
//

#include "predicate.h"

// define static variable for statistics
PredicateStatistics Predicate::statistics;

bool Predicate::statisticBooleanEvaluation(int timeStep,
                                           const std::shared_ptr<World>& world,
                                           const std::shared_ptr<Obstacle>& obstacleK,
                                           const std::shared_ptr<Obstacle>& obstacleP) {
    statistics.numExecutions++;
    return booleanEvaluation(timeStep, world, obstacleK, obstacleP);
}

bool Predicate::booleanEvaluation(int timeStep,
                                  const std::shared_ptr<World> &world,
                                  const std::shared_ptr<Obstacle> &obstacleK,
                                  const std::shared_ptr<Obstacle> &obstacleP) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP) > 0;
}

const PredicateParameters &Predicate::getParameters() const { return parameters; }

void Predicate::setParameters(const PredicateParameters &params) { parameters = params; }

Predicate::Predicate(const PredicateParameters &parameters) : parameters(parameters) {}

Predicate::Predicate() = default;
