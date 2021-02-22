//
// Created by Sebastian Maierhofer on 19.02.21.
//

#include "predicate.h"

// define static variable for statistics
PredicateStatistics Predicate::statistics;

bool Predicate::statisticBooleanEvaluation(int timeStep,
                                           const std::shared_ptr<RoadNetwork>& roadNetwork,
                                           const std::shared_ptr<Obstacle>& obstacleK,
                                           const std::shared_ptr<Obstacle>& obstacleP,
                                           const std::vector<std::shared_ptr<Obstacle>>& obstacles) {
    statistics.numExecutions++;
    return booleanEvaluation(timeStep, roadNetwork, obstacleK, obstacleP, obstacles);
}

const PredicateParameters &Predicate::getParameters() const { return parameters; }

void Predicate::setParameters(const PredicateParameters &params) { parameters = params; }

const PredicateStatistics &Predicate::getStatistics() { return statistics; }
