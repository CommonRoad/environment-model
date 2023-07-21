#include "commonroad_cpp/predicates/general/is_vehicle_predicate.h"

IsVehiclePredicate::IsVehiclePredicate() : CommonRoadPredicate(false) {}

bool IsVehiclePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return obstacleK->getObstacleType() == ObstacleType::car or obstacleK->getObstacleType() == ObstacleType::bus or
           obstacleK->getObstacleType() == ObstacleType::motorcycle or
           obstacleK->getObstacleType() == ObstacleType::priority_vehicle or
           obstacleK->getObstacleType() == ObstacleType::taxi or obstacleK->getObstacleType() == ObstacleType::bicycle;
}

double
IsVehiclePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                     const std::shared_ptr<Obstacle> &obstacleK,
                                     const std::shared_ptr<Obstacle> &obstacleP,
                                     const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("IsVehiclePredicate does not support robust evaluation!");
}

Constraint IsVehiclePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("IsVehiclePredicate does not support constraint evaluation!");
}
