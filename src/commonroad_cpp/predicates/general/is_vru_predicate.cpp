#include "commonroad_cpp/predicates/general/is_vru_predicate.h"

IsVruPredicate::IsVruPredicate() : CommonRoadPredicate(false) {}

bool IsVruPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return obstacleK->getObstacleType() == ObstacleType::bicycle or
           obstacleK->getObstacleType() == ObstacleType::pedestrian or
           obstacleK->getObstacleType() == ObstacleType::motorcycle;
}

double
IsVruPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                 const std::shared_ptr<Obstacle> &obstacleK, const std::shared_ptr<Obstacle> &obstacleP,
                                 const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return 0;
}

Constraint
IsVruPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                     const std::shared_ptr<Obstacle> &obstacleK,
                                     const std::shared_ptr<Obstacle> &obstacleP,
                                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return Constraint();
}
