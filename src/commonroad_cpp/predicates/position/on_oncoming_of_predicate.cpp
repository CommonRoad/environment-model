//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "on_oncoming_of_predicate.h"
#include "../../geometry/geometric_operations.h"
#include "../../obstacle/obstacle.h"
#include "../../roadNetwork/intersection/incoming.h"
#include "../../roadNetwork/lanelet/lane.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <cmath>
#include <commonroad_cpp/world.h>

bool OnOncomingOfPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {

    auto intersections{obstacle_operations::getIntersections(timeStep, world->getRoadNetwork(), obstacleP)};
    std::vector<std::shared_ptr<Incoming>> incomings;
    for (const auto &inter : intersections)
        for (const auto &incom : inter->getIncomings()) {
            auto angle{incom->getIncomingLanelets().at(0)->getOrientation().back()};
            auto angleDif{M_PI - std::abs(geometric_operations::subtractOrientations(
                                     angle, obstacleP->getStateByTimeStep(timeStep)->getGlobalOrientation()))};
            if (angleDif > -0.3 and angleDif < 0.3)
                incomings.push_back(incom);
        }
    auto lanelets{obstacle_operations::getSimilarlyOrientedLanelets(
        world->getRoadNetwork(), obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep),
        obstacleK->getStateByTimeStep(timeStep), parameters.laneletOccupancySimilarity)};
    for (const auto &let : lanelets)
        for (const auto &incom : incomings) {
            auto straightSuccessors{incom->getAllSuccessorStraight()};
            if (std::any_of(straightSuccessors.begin(), straightSuccessors.end(),
                            [let](const std::shared_ptr<Lanelet> &letSuc) { return let->getId() == letSuc->getId(); }))
                return true;
        }
    return false;
}

double OnOncomingOfPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnIncomingLeftOfPredicate does not support robust evaluation!");
}

Constraint OnOncomingOfPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnIncomingLeftOfPredicate does not support constraint evaluation!");
}
OnOncomingOfPredicate::OnOncomingOfPredicate() : CommonRoadPredicate(true) {}
