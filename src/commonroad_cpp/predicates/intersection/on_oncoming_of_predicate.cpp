#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include <cmath>
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/on_oncoming_of_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool OnOncomingOfPredicate::booleanEvaluation(const size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters,
                                              const bool setBased) {

    const auto intersections{obstacle_operations::getIntersections(timeStep, world->getRoadNetwork(), obstacleP)};
    std::vector<std::shared_ptr<IncomingGroup>> incomings;
    for (const auto &inter : intersections)
        for (const auto &incom : inter->getIncomingGroups()) {
            const auto angle{incom->getIncomingLanelets().at(0)->getOrientation().back()};
            const auto angleDif{M_PI - std::abs(geometric_operations::subtractOrientations(
                                           angle, obstacleP->getStateByTimeStep(timeStep)->getGlobalOrientation()))};
            if (std::abs(angleDif) < std::stod(additionalFunctionParameters.at(0)))
                incomings.push_back(incom);
        }
    const auto lanelets{
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, setBased)};
    for (const auto &let : lanelets)
        for (const auto &incom : incomings) {
            if (auto straightSuccessors{incom->getAllStraightGoingLanelets()};
                std::any_of(straightSuccessors.begin(), straightSuccessors.end(),
                            [let](const std::shared_ptr<Lanelet> &letSuc) { return let->getId() == letSuc->getId(); }))
                return true;
            if (auto rightSuccessors{incom->getAllRightTurningLanelets()};
                std::any_of(rightSuccessors.begin(), rightSuccessors.end(),
                            [let](const std::shared_ptr<Lanelet> &letSuc) { return let->getId() == letSuc->getId(); }))
                return true;
            if (auto incomingLanelets{incom->getIncomingLanelets()};
                std::any_of(incomingLanelets.begin(), incomingLanelets.end(),
                            [let](const std::shared_ptr<Lanelet> &letSuc) { return let->getId() == letSuc->getId(); }))
                return true;
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
