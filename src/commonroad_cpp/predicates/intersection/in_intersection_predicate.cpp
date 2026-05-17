#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/in_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool InIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters,
                                                bool setBased) {
    // Cache parsed ID; params[0] is fixed per scenario instance. Saves some resource compared to using std::stoul at
    // every function call
    if (additionalFunctionParameters[0] != cachedIdStr_) {
        cachedIdStr_ = additionalFunctionParameters[0];
        cachedId_ = std::stoul(additionalFunctionParameters[0]);
    }
    const auto intersectionId = cachedId_;
    const auto &roadNetwork = world->getRoadNetwork();
    const auto lanelets = obstacleK->getOccupiedLaneletsByShape(roadNetwork, timeStep);
    const auto intersection = roadNetwork->getIntersectionByID(intersectionId);
    return std::find_if(lanelets.begin(), lanelets.end(), [&intersection](const std::shared_ptr<Lanelet> &la) {
               return la->hasLaneletType(LaneletType::intersection) and intersection->isMemberLanelet(la->getId());
           }) != lanelets.end();
}

double InIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    throw std::runtime_error("InIntersectionPredicate does not support robust evaluation!");
}

Constraint InIntersectionPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters,
                                                         bool setBased) {
    throw std::runtime_error("InIntersectionPredicate does not support constraint evaluation!");
}
InIntersectionPredicate::InIntersectionPredicate() : CommonRoadPredicate(false) {}
