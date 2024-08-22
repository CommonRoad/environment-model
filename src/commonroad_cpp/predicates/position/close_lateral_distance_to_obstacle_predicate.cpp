#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"
#include "commonroad_cpp/world.h"

#include "commonroad_cpp/predicates/position/close_lateral_distance_to_obstacle_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle_operations.h>

bool CloseLateralDistanceToObstaclePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    auto leftP{obstacleP->leftD(timeStep, obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep))};
    auto rightP{obstacleP->rightD(timeStep, obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep))};
    if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::right) {
        auto rightK{obstacleK->rightD(world->getRoadNetwork(), timeStep)};
        auto maxP{std::max(leftP, rightP)}; // different driving directions / orientations
        return rightK > maxP and (rightK - maxP) <= parameters.getParam("closeToOtherVehicle");
    } else if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::left) {
        auto leftK{obstacleK->leftD(world->getRoadNetwork(), timeStep)};
        auto minP{std::min(leftP, rightP)}; // different driving directions / orientations
        return minP > leftK and (minP - leftK) < parameters.getParam("closeToOtherVehicle");
    }
    throw std::runtime_error("CloseLateralDistanceToObstaclePredicate::booleanEvaluation: Unknown side '" +
                             additionalFunctionParameters.at(0) + "'!");
}

double CloseLateralDistanceToObstaclePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CloseLateralDistanceToObstaclePredicate does not support robust evaluation!");
}

Constraint CloseLateralDistanceToObstaclePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("CloseLateralDistanceToObstaclePredicate does not support constraint evaluation!");
}

CloseLateralDistanceToObstaclePredicate::CloseLateralDistanceToObstaclePredicate() : CommonRoadPredicate(false) {}
