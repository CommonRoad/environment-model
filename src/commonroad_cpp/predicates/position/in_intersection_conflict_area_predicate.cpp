//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/position/in_intersection_conflict_area_predicate.h>
#include <commonroad_cpp/predicates/position/on_lanelet_with_type_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

bool InIntersectionConflictAreaPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    OnLaneletWithTypePredicate onLaneletWithTypePredicate;
    std::shared_ptr<OptionalPredicateParameters> opt{
        std::make_shared<OptionalPredicateParameters>(std::vector<LaneletType>{LaneletType::intersection})};
    if (!onLaneletWithTypePredicate.booleanEvaluation(timeStep, world, obstacleK, obstacleP, opt))
        return false;
    auto simLaneletsK{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep)};
    std::shared_ptr<Lane> lane{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};

    for (const auto &letP : lane->getContainedLanelets()) {
        for (const auto &letK : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)) {
            if (!letK->hasLaneletType(LaneletType::intersection))
                continue;
            if (letK->getId() == letP->getId() and
                !std::any_of(
                    simLaneletsK.begin(), simLaneletsK.end(),
                    [letK](const std::shared_ptr<Lanelet> &letSim) { return letSim->getId() == letK->getId(); }) and
                !std::any_of(simLaneletsK.begin(), simLaneletsK.end(),
                             [letP, world](const std::shared_ptr<Lanelet> &letSim) {
                                 return intersection_operations::checkSameIncoming(letP, letSim);
                             }))
                return true;
        }
    }

    return false;
}

double InIntersectionConflictAreaPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support robust evaluation!");
}

Constraint InIntersectionConflictAreaPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP,
    const std::shared_ptr<OptionalPredicateParameters> &additionalFunctionParameters) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support constraint evaluation!");
}

InIntersectionConflictAreaPredicate::InIntersectionConflictAreaPredicate() : CommonRoadPredicate(true) {}