#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/decelerates_predicate.h>
#include <commonroad_cpp/world.h>

bool DeceleratesPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP) < additionalFunctionParameters->acceleration;
}

Constraint DeceleratesPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("DeceleratesPredicate does not support constraint evaluation!");
}

double DeceleratesPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return obstacleK->getStateByTimeStep(timeStep)->getAcceleration();
}

DeceleratesPredicate::DeceleratesPredicate() : CommonRoadPredicate(false) {}
