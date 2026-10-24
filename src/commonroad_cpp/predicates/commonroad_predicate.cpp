#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/predicates/commonroad_predicate.h>

bool CommonRoadPredicate::statisticBooleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Timer> &evaluationTimer,
                                                     const std::shared_ptr<PredicateStatistics> &statistics,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     const bool setBased) {
    const auto startTime{Timer::start()};
    const bool result{booleanEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters, setBased)};
    const long compTime{evaluationTimer->stop(startTime)};

    statistics->numExecutions++;
    statistics->totalComputationTime += static_cast<unsigned long>(compTime);
    statistics->computationTime.push_back(static_cast<double>(compTime) / 1e6);
    if (result)
        statistics->numSatisfaction++;

    return result;
}

PredicateParameters &CommonRoadPredicate::getParameters() { return parameters; }

void CommonRoadPredicate::setParameters(const PredicateParameters &params) { parameters = params; }

CommonRoadPredicate::CommonRoadPredicate(const bool vehicleDependent) : vehicleDependent(vehicleDependent) {}

bool CommonRoadPredicate::simpleBooleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  const bool setBased) {
    return this->booleanEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters, setBased);
}

CommonRoadPredicate::~CommonRoadPredicate() = default;

bool CommonRoadPredicate::isVehicleDependent() const { return vehicleDependent; }

bool CommonRoadPredicate::evaluatesOccupancy(const std::shared_ptr<Obstacle> &obstacle, const size_t timeStep,
                                             const bool setBased) {
    return setBased and obstacle != nullptr and !obstacle->getSetBasedPrediction().empty() and
           timeStep > obstacle->getCurrentState()->getTimeStep();
}
