#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/approach_intersection_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool ApproachIntersectionPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
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
    const auto lanelets = obstacleK->getOccupiedLaneletsByShape(roadNetwork, timeStep);
    const auto intersection = roadNetwork->getIntersectionByID(intersectionId);
    bool approachIntersection = false;
    for (const auto &lanelet : lanelets) {
        if (!intersection->isMemberLanelet(lanelet->getId()))
            continue;
        if (lanelet->hasLaneletType(LaneletType::intersection))
            return false;
        if (lanelet->hasLaneletType(LaneletType::incoming))
            approachIntersection = true;
    }
    return approachIntersection;
}

double ApproachIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("ApproachIntersectionPredicate does not support robust evaluation!");
}

Constraint ApproachIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("ApproachIntersectionPredicate does not support constraint evaluation!");
}
ApproachIntersectionPredicate::ApproachIntersectionPredicate() : CommonRoadPredicate(false) {}
