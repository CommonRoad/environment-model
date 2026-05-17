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
    // Cache parsed ID; params[0] is fixed per scenario instance. Saves some resource compared to using std::stoul at
    // every function call.
    if (additionalFunctionParameters[0] != cachedIdStr_) {
        cachedIdStr_ = additionalFunctionParameters[0];
        cachedId_ = std::stoul(additionalFunctionParameters[0]);
    }
    const auto intersectionId = cachedId_;
    const auto &roadNetwork = world->getRoadNetwork();
    const auto laneletsK = obstacleK->getOccupiedLaneletsByShape(roadNetwork, timeStep);
    const auto intersection = roadNetwork->getIntersectionByID(intersectionId);
    return std::any_of(laneletsK.begin(), laneletsK.end(), [&intersection](const std::shared_ptr<Lanelet> &lanelet) {
        return lanelet->hasLaneletType(LaneletType::incoming) and intersection->isMemberLanelet(lanelet->getId());
    });
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
