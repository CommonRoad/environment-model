#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_outermost_lane_predicate.h>
#include <commonroad_cpp/world.h>

bool InOutermostLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    return std::any_of(
        lanelets.begin(), lanelets.end(), [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
            if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::left)
                return lanelet->getAdjacentLeft().adj == nullptr ||
                       lanelet->getAdjacentLeft().oppositeDir !=
                           lanelet->getAdjacentLeft().adj->getAdjacentRight().oppositeDir;
            if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::right)
                return lanelet->getAdjacentRight().adj == nullptr ||
                       lanelet->getAdjacentRight().oppositeDir !=
                           lanelet->getAdjacentRight().adj->getAdjacentLeft().oppositeDir;
            throw std::runtime_error("InOutermostLanePredicate::booleanEvaluation: Invalid turning direction.");
        });
}

double InOutermostLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InOutermostLanePredicate does not support robust evaluation!");
}

Constraint InOutermostLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InOutermostLanePredicate does not support constraint evaluation!");
}

InOutermostLanePredicate::InOutermostLanePredicate() : CommonRoadPredicate(false) {}
