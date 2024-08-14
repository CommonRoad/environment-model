#include <commonroad_cpp/predicates/commonroad_predicate.h>

bool CommonRoadPredicate::statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Timer> &evaluationTimer,
                                                     const std::shared_ptr<PredicateStatistics> &statistics,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters) {
    auto startTime{Timer::start()};
    bool result{booleanEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters)};
    long compTime{evaluationTimer->stop(startTime)};

    statistics->numExecutions++;
    statistics->totalComputationTime += static_cast<unsigned long>(compTime);
    statistics->computationTime.push_back(static_cast<double>(compTime) / 1e6);
    if (result)
        statistics->numSatisfaction++;

    return result;
}

PredicateParameters &CommonRoadPredicate::getParameters() { return parameters; }

void CommonRoadPredicate::setParameters(const PredicateParameters &params) { parameters = params; }

CommonRoadPredicate::CommonRoadPredicate(bool vehicleDependent) : vehicleDependent(vehicleDependent) {}

bool CommonRoadPredicate::simpleBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP) {
    return this->booleanEvaluation(timeStep, world, obstacleK, obstacleP);
}

CommonRoadPredicate::~CommonRoadPredicate() {}

bool CommonRoadPredicate::isVehicleDependent() const { return vehicleDependent; }
