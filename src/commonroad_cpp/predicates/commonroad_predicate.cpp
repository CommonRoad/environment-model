//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <commonroad_cpp/predicates/commonroad_predicate.h>

#include <utility>

bool CommonRoadPredicate::statisticBooleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    auto startTime{Timer::start()};
    bool result{booleanEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters)};
    long compTime{evaluationTimer.stop(startTime)};

    // TODO Thread-local storage for stats?
    omp_set_lock(&writelock);
    {
        statistics.numExecutions++;
        statistics.totalComputationTime += static_cast<unsigned long>(compTime);
        if (compTime > statistics.maxComputationTime)
            statistics.maxComputationTime = compTime;
        if (static_cast<unsigned long>(compTime) < statistics.minComputationTime)
            statistics.minComputationTime = static_cast<size_t>(compTime);
        if (result)
            statistics.numSatisfaction++;
        omp_unset_lock(&writelock);
    }
    return result;
}

const PredicateParameters &CommonRoadPredicate::getParameters() const { return parameters; }

void CommonRoadPredicate::setParameters(const PredicateParameters &params) { parameters = params; }

const PredicateStatistics &CommonRoadPredicate::getStatistics() const { return statistics; }

void CommonRoadPredicate::resetStatistics() {
    statistics.minComputationTime = LONG_MAX;
    statistics.maxComputationTime = LONG_MIN;
    statistics.totalComputationTime = 0;
    statistics.numSatisfaction = 0;
    statistics.numExecutions = 0;
}

const Timer &CommonRoadPredicate::getEvaluationTimer() const { return evaluationTimer; }

CommonRoadPredicate::CommonRoadPredicate(bool vehicleDependent) : vehicleDependent(vehicleDependent) {
    omp_init_lock(&writelock);
}

bool CommonRoadPredicate::simpleBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP) {
    return this->booleanEvaluation(timeStep, world, obstacleK, obstacleP);
}

CommonRoadPredicate::~CommonRoadPredicate() { omp_destroy_lock(&writelock); }

bool CommonRoadPredicate::isVehicleDependent() const { return vehicleDependent; }

OptionalPredicateParameters::OptionalPredicateParameters(std::vector<TrafficSignTypes> signType)
    : signType(std::move(signType)) {}
OptionalPredicateParameters::OptionalPredicateParameters(std::vector<LaneletType> laneletType)
    : laneletType(std::move(laneletType)) {}
OptionalPredicateParameters::OptionalPredicateParameters(std::vector<TurningDirection> turningDirection)
    : turningDirection(std::move(turningDirection)) {}
OptionalPredicateParameters::OptionalPredicateParameters(std::vector<TrafficSignTypes> signType,
                                                         std::vector<LaneletType> laneletType,
                                                         std::vector<TurningDirection> turningDirection,
                                                         std::vector<TrafficLightState> trafficLightState)
    : signType(std::move(signType)), laneletType(std::move(laneletType)), turningDirection(std::move(turningDirection)),
      trafficLightState(std::move(trafficLightState)) {}
OptionalPredicateParameters::OptionalPredicateParameters(std::vector<TrafficLightState> trafficLightState)
    : trafficLightState(std::move(trafficLightState)) {}
OptionalPredicateParameters::OptionalPredicateParameters(double minSafetyDistance)
    : minSafetyDistance(minSafetyDistance) {}
