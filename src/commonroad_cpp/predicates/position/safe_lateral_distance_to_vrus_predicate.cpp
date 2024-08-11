#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/safe_lateral_distance_to_vrus_predicate.h>
#include <commonroad_cpp/world.h>

SafeLateralDistanceToVrusPredicate::SafeLateralDistanceToVrusPredicate() : CommonRoadPredicate(true) {}

bool SafeLateralDistanceToVrusPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {

    auto lanelets = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    double dist = obstacleK->getLateralDistanceToObstacle(timeStep, obstacleP, world->getRoadNetwork());
    if (dist < parameters.getParam("dMinUrban") or (!std::any_of(lanelets.begin(), lanelets.end(),
                                                                 [](const std::shared_ptr<Lanelet> &lanelet) {
                                                                     return lanelet->hasLaneletType(LaneletType::urban);
                                                                 }) and
                                                    dist < parameters.getParam("dMinNonUrban")))
        return false;
    return true;
}

double SafeLateralDistanceToVrusPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("SafeLateralDistanceToVrusPredicate does not support robust evaluation!");
}

Constraint SafeLateralDistanceToVrusPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("SafeLateralDistanceToVrusPredicate does not support constraint evaluation!");
}
