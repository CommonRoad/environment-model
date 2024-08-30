#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/safe_lateral_distance_to_vrus_predicate.h>
#include <commonroad_cpp/world.h>

SafeLateralDistanceToVrusPredicate::SafeLateralDistanceToVrusPredicate() : CommonRoadPredicate(true) {}

bool SafeLateralDistanceToVrusPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    auto dist{obstacleK->getLateralDistanceToObstacle(timeStep, obstacleP, world->getRoadNetwork())};
    return dist >= std::stod(additionalFunctionParameters.at(0));
}

double SafeLateralDistanceToVrusPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("SafeLateralDistanceToVrusPredicate does not support robust evaluation!");
}

Constraint SafeLateralDistanceToVrusPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("SafeLateralDistanceToVrusPredicate does not support constraint evaluation!");
}
