#include "commonroad_cpp/predicates/general/left_signal_set_predicate.h"

#include <commonroad_cpp/obstacle/obstacle.h>

LeftSignalSetPredicate::LeftSignalSetPredicate() : CommonRoadPredicate(false) {}

bool LeftSignalSetPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters) {
    return obstacleK->getSignalStateByTimeStep(timeStep)->isIndicatorLeft();
}

double LeftSignalSetPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("Left signal set Predicate does not support robust evaluation!");
}

Constraint LeftSignalSetPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("Left signal set Predicate does not support constraint evaluation!");
}
