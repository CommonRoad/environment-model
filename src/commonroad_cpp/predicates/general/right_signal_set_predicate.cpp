#include "commonroad_cpp/predicates/general/right_signal_set_predicate.h"

#include "commonroad_cpp/obstacle/obstacle.h"

RightSignalSetPredicate::RightSignalSetPredicate() : CommonRoadPredicate(false) {}

bool RightSignalSetPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return obstacleK->getSignalStateByTimeStep(timeStep)->isIndicatorRight();
}

double RightSignalSetPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Right signal set Predicate does not support robust evaluation!");
}

Constraint RightSignalSetPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Right signal set Predicate does not support constraint evaluation!");
}
