#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/in_neighboring_left_lane_predicate.h>

bool InNeighboringLeftLanePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    std::unordered_set<unsigned long> relevantIDs;
    auto laneletsP = obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &laneK : obstacleK->getDrivingPathLanes(world->getRoadNetwork(), timeStep)) {
        for (const auto &laneletK : laneK->getContainedLanelets()) {
            if (laneletK->getAdjacentLeft().adj and
                std::any_of(laneletsP.begin(), laneletsP.end(), [laneletK](const std::shared_ptr<Lanelet> &lanelet) {
                    return laneletK->getAdjacentLeft().adj->getId() == lanelet->getId();
                })) {
                return true;
            }
        }
    }
    return false;
}

double InNeighboringLeftLanePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Neighboring Lane Predicate does not support robust evaluation!");
}

Constraint InNeighboringLeftLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("In Neighboring Lane Predicate does not support constraint evaluation!");
}
InNeighboringLeftLanePredicate::InNeighboringLeftLanePredicate() : CommonRoadPredicate(true) {}
