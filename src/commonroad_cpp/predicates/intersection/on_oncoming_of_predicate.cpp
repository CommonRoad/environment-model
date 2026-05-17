#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include <cmath>
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/on_oncoming_of_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <unordered_set>

bool OnOncomingOfPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters,
                                              const bool setBased) {

    if (additionalFunctionParameters.at(0) != cachedAngleToleranceStr_) {
        cachedAngleToleranceStr_ = additionalFunctionParameters.at(0);
        cachedAngleTolerance_ = std::stod(cachedAngleToleranceStr_);
    }
    const auto angleTolerance = cachedAngleTolerance_;
    const auto intersections{obstacle_operations::getIntersections(timeStep, world->getRoadNetwork(), obstacleP)};
    std::vector<std::shared_ptr<IncomingGroup>> incomings;
    for (const auto &inter : intersections)
        for (const auto &incom : inter->getIncomingGroups()) {
            const auto angle{incom->getIncomingLanelets().at(0)->getOrientation().back()};
            const auto angleDif{M_PI - std::abs(geometric_operations::subtractOrientations(
                                           angle, obstacleP->getStateByTimeStep(timeStep)->getGlobalOrientation()))};
            if (std::abs(angleDif) < angleTolerance)
                incomings.push_back(incom);
        }
    const auto lanelets{
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, setBased)};
    auto buildIdSet = [](const std::vector<std::shared_ptr<Lanelet>> &v) {
        std::unordered_set<size_t> s;
        for (const auto &l : v)
            s.insert(l->getId());
        return s;
    };
    for (const auto &incom : incomings) {
        const auto straightIds{buildIdSet(incom->getAllStraightGoingLanelets())};
        const auto rightIds{buildIdSet(incom->getAllRightTurningLanelets())};
        const auto incomingIds{buildIdSet(incom->getIncomingLanelets())};
        for (const auto &let : lanelets) {
            const auto id = let->getId();
            if (straightIds.count(id) or rightIds.count(id) or incomingIds.count(id))
                return true;
        }
    }
    return false;
}

double OnOncomingOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters,
                                               bool setBased) {
    throw std::runtime_error("OnOncomingOfPredicate does not support robust evaluation!");
}

Constraint OnOncomingOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("OnOncomingOfPredicate does not support constraint evaluation!");
}
OnOncomingOfPredicate::OnOncomingOfPredicate() : CommonRoadPredicate(true) {}
