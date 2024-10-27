#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/lane/in_neighboring_lane_predicate.h>

bool InNeighboringLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters) {
    std::unordered_set<unsigned long> relevantIDs;
    auto laneletsP = obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &laneK : obstacleK->getOccupiedRoadLanes(world->getRoadNetwork(), timeStep)) {
        for (const auto &laneletK : laneK->getContainedLanelets()) {
            if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::left and
                laneletK->getAdjacentLeft().adj and
                std::any_of(laneletsP.begin(), laneletsP.end(), [laneletK](const std::shared_ptr<Lanelet> &lanelet) {
                    return laneletK->getAdjacentLeft().adj->getId() == lanelet->getId();
                })) {
                return true;
            }
            if (TrafficLight::matchTurningDirections(additionalFunctionParameters.at(0)) == TurningDirection::right and
                laneletK->getAdjacentRight().adj and
                std::any_of(laneletsP.begin(), laneletsP.end(), [laneletK](const std::shared_ptr<Lanelet> &lanelet) {
                    return laneletK->getAdjacentRight().adj->getId() == lanelet->getId();
                })) {
                return true;
            }
        }
    }
    return false;
}

double InNeighboringLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InNeighboringLanePredicate does not support robust evaluation!");
}

Constraint InNeighboringLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("InNeighboringLanePredicate does not support constraint evaluation!");
}
InNeighboringLanePredicate::InNeighboringLanePredicate() : CommonRoadPredicate(true) {}
