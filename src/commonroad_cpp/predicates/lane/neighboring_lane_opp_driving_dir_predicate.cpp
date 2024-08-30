#include "commonroad_cpp/predicates/lane/neighboring_lane_opp_driving_dir_predicate.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

bool NeighboringLaneOppDrivingDirPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    auto occLanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::left)
        return std::any_of(occLanelets.begin(), occLanelets.end(),
                           [](const std::shared_ptr<Lanelet> &la) { return la->getAdjacentLeft().oppositeDir; });
    else
        return std::any_of(occLanelets.begin(), occLanelets.end(),
                           [](const std::shared_ptr<Lanelet> &la) { return la->getAdjacentRight().oppositeDir; });
}

double NeighboringLaneOppDrivingDirPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("NeighboringLaneOppDrivingDirPredicate does not support robust evaluation!");
}

Constraint NeighboringLaneOppDrivingDirPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("NeighboringLaneOppDrivingDirPredicate does not support constraint evaluation!");
}
NeighboringLaneOppDrivingDirPredicate::NeighboringLaneOppDrivingDirPredicate() : CommonRoadPredicate(false) {}
