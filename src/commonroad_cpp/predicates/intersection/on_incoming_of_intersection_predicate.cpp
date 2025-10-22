#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/on_incoming_of_intersection_predicate.h>
#include <commonroad_cpp/world.h>

bool OnIncomingOfIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const std::vector<std::string> &additionalFunctionParameters,
                                                          bool setBased) {
    std::vector<std::shared_ptr<Lanelet>> laneletsK =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    if (std::any_of(laneletsK.begin(), laneletsK.end(),
                    [world, additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                        return lanelet->hasLaneletType(LaneletType::incoming) and
                               world->getRoadNetwork()
                                   ->getIntersectionByID(std::stoul(additionalFunctionParameters[0]))
                                   ->isMemberLanelet(lanelet->getId());
                    }))
        return true;
    return false;
}

double OnIncomingOfIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           const std::vector<std::string> &additionalFunctionParameters,
                                                           bool setBased) {
    throw std::runtime_error("OnIncomingOfIntersectionPredicate does not support robust evaluation!");
}

Constraint OnIncomingOfIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OnIncomingOfIntersectionPredicate does not support constraint evaluation!");
}
OnIncomingOfIntersectionPredicate::OnIncomingOfIntersectionPredicate() : CommonRoadPredicate(false) {}
