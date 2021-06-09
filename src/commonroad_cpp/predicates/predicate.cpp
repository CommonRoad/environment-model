//
// Created by Sebastian Maierhofer on 19.02.21.
//

#include "predicate.h"

bool CommonRoadPredicate::statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
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

const PredicateParameters &CommonRoadPredicate::getParameters() const { return parameters; }

void CommonRoadPredicate::setParameters(const PredicateParameters &params) { parameters = params; }

CommonRoadPredicate::CommonRoadPredicate(const PredicateParameters &parameters) : parameters(parameters) {}

const PredicateStatistics &CommonRoadPredicate::getStatistics() const { return statistics; }

const Timer &CommonRoadPredicate::getEvaluationTimer() const { return evaluationTimer; }

CommonRoadPredicate::CommonRoadPredicate() = default;
