#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_neighboring_lane_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <unordered_set>

bool InNeighboringLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    auto laneletsP = obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    std::unordered_set<size_t> laneletPIDs;
    for (const auto &la : laneletsP)
        laneletPIDs.insert(la->getId());
    const auto direction = regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0));
    for (const auto &laneK : obstacleK->getOccupiedRoadLanes(world->getRoadNetwork(), timeStep)) {
        for (const auto &laneletK : laneK->getContainedLanelets()) {
            auto adjacent{laneletK->getAdjacent(direction).adj};
            if (adjacent and laneletPIDs.count(adjacent->getId())) {
                return true;
            }
        }
    }
    return false;
}

double InNeighboringLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    throw std::runtime_error("InNeighboringLanePredicate does not support robust evaluation!");
}

Constraint InNeighboringLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("InNeighboringLanePredicate does not support constraint evaluation!");
}
InNeighboringLanePredicate::InNeighboringLanePredicate() : CommonRoadPredicate(true) {}
