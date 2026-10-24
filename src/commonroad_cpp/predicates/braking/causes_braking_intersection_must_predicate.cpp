#include <commonroad_cpp/predicates/braking/causes_braking_intersection_must_predicate.h>

bool CausesBrakingIntersectionMustPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    return evaluateSetBased(timeStep, world, obstacleK, obstacleP, setBased, true);
}
