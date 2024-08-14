#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/in_neighboring_right_lane_predicate.h>

bool InNeighboringRightLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters) {
    std::unordered_set<unsigned long> relevantIDs;
    auto laneletsP = obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &laneK : obstacleK->getOccupiedRoadLanes(world->getRoadNetwork(), timeStep)) {
        for (const auto &laneletK : laneK->getContainedLanelets()) {
            if (laneletK->getAdjacentRight().adj and
                std::any_of(laneletsP.begin(), laneletsP.end(), [laneletK](const std::shared_ptr<Lanelet> &lanelet) {
                    return laneletK->getAdjacentRight().adj->getId() == lanelet->getId();
                })) {
                return true;
            }
        }
    }
    return false;
}

double InNeighboringRightLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("In Neighboring Lane Predicate does not support robust evaluation!");
}

Constraint InNeighboringRightLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters) {
    throw std::runtime_error("In Neighboring Lane Predicate does not support constraint evaluation!");
}
InNeighboringRightLanePredicate::InNeighboringRightLanePredicate() : CommonRoadPredicate(true) {}
