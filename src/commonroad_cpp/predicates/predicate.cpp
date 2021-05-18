//
// Created by Sebastian Maierhofer on 19.02.21.
//

#include "predicate.h"

bool Predicate::statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::shared_ptr<Obstacle> &obstacleP) {
    statistics.numExecutions++;
    auto startTime{Timer::start()};
    bool result{booleanEvaluation(timeStep, world, obstacleK, obstacleP)};
    long compTime{evaluationTimer.stop(startTime)};
    if (compTime > statistics.maxComputationTime)
        statistics.maxComputationTime = compTime;
    else if (compTime > statistics.minComputationTime)
        statistics.minComputationTime = compTime;
    else if (result)
        statistics.numSatisfaction++;
    return result;
}

const PredicateParameters &Predicate::getParameters() const { return parameters; }

void Predicate::setParameters(const PredicateParameters &params) { parameters = params; }

Predicate::Predicate(const PredicateParameters &parameters) : parameters(parameters) {}

Predicate::Predicate() = default;
