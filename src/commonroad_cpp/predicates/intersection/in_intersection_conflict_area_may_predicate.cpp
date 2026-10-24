#include <commonroad_cpp/predicates/intersection/in_intersection_conflict_area_may_predicate.h>

bool InIntersectionConflictAreaMayPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    return evaluateSetBased(timeStep, world, obstacleK, obstacleP, setBased, false);
}
