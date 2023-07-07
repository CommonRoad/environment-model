//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/on_incoming_left_of_predicate.h>
#include <commonroad_cpp/predicates/position/on_lanelet_with_type_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/incoming_group.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool OnIncomingLeftOfPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {

    OnLaneletWithTypePredicate onLaneletWithTypePredicate;
    std::shared_ptr<OptionalPredicateParameters> opt{
        std::make_shared<OptionalPredicateParameters>(std::vector<LaneletType>{LaneletType::incoming})};
    if (!onLaneletWithTypePredicate.booleanEvaluation(timeStep, world, obstacleK, {}, opt) or
        !onLaneletWithTypePredicate.booleanEvaluation(timeStep, world, obstacleP, {}, opt))
        return false;

    auto laneK = obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep);
    auto laneP = obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep);
    for (const auto &letK : laneK->getContainedLanelets()) {
        if (!letK->hasLaneletType(LaneletType::incoming))
            continue;
        auto incomingK{world->getRoadNetwork()->findIncomingGroupByLanelet(letK)};
        if (!incomingK) {
            throw std::runtime_error{"missing incoming"};
        }
        for (const auto &letP : laneP->getContainedLanelets()) {
            if (!letP->hasLaneletType(LaneletType::incoming))
                continue;
            auto incomingP{world->getRoadNetwork()->findIncomingGroupByLanelet(letP)};
            if (!incomingP) {
                throw std::runtime_error{"missing incoming"};
            }
            if (!incomingP->getIsLeftOf()) {
                throw std::runtime_error{"missing incoming"};
            }
            if (incomingK->getIsLeftOf()->getId() == incomingP->getId())
                return true;
        };
    }
    return false;
}

double OnIncomingLeftOfPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnIncomingLeftOfPredicate does not support robust evaluation!");
}

Constraint OnIncomingLeftOfPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("OnIncomingLeftOfPredicate does not support constraint evaluation!");
}
OnIncomingLeftOfPredicate::OnIncomingLeftOfPredicate() : CommonRoadPredicate(true) {}
