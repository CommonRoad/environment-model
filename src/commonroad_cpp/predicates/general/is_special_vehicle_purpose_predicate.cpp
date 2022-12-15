#include "commonroad_cpp/predicates/general/is_special_vehicle_purpose_predicate.h"

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/obstacle/obstacle.h>

isSpecialVehiclePurposePredicate::isSpecialVehiclePurposePredicate() : CommonRoadPredicate(false) {}

bool isSpecialVehiclePurposePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    return obstacleK->getObstacleType() == ObstacleType::bus || obstacleK->getObstacleType() == ObstacleType::taxi ||
           obstacleK->getObstacleType() == ObstacleType::bicycle;
}

double isSpecialVehiclePurposePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Is special vehicle purpose Predicate does not support robust evaluation!");
}

Constraint isSpecialVehiclePurposePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("Is special vehicle purpose Predicate does not support constraint evaluation!");
}
