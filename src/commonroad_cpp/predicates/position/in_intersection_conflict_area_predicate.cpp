//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "in_intersection_conflict_area_predicate.h"
#include "../../obstacle/obstacle.h"
#include "../../obstacle/obstacle_operations.h"
#include "../../roadNetwork/lanelet/lane.h"
#include "../../roadNetwork/lanelet/lanelet_operations.h"
#include "../../world.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "on_lanelet_with_type_predicate.h"

std::vector<std::shared_ptr<Lanelet>> combineLaneLanelets(const std::vector<std::shared_ptr<Lane>> &lanes) {
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &lane : lanes)
        for (const auto &let : lane->getContainedLanelets())
            if (!(std::any_of(lanelets.begin(), lanelets.end(),
                              [let](const std::shared_ptr<Lanelet> &exLet) { return exLet->getId() == let->getId(); })))
                lanelets.push_back(let);
    return lanelets;
}

std::vector<std::shared_ptr<Lanelet>>
combineLaneLanelets(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &lanes) {
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &lane : lanes)
        for (const auto &let : lane)
            if (!(std::any_of(lanelets.begin(), lanelets.end(),
                              [let](const std::shared_ptr<Lanelet> &exLet) { return exLet->getId() == let->getId(); })))
                lanelets.push_back(let);
    return lanelets;
}

bool checkSameIncoming(const std::shared_ptr<Lanelet> &letk, const std::shared_ptr<Lanelet> &letp,
                       const std::shared_ptr<RoadNetwork> &roadNetwork) {
    auto simLaneletsK{combineLaneLanelets(lanelet_operations::combineLaneletAndPredecessorsToLane(letk))};
    auto simLaneletsP{combineLaneLanelets(lanelet_operations::combineLaneletAndPredecessorsToLane(letp))};
    for (const auto &laK : simLaneletsK) {
        if (!laK->hasLaneletType(LaneletType::incoming))
            continue;
        for (const auto &adjLet : lanelet_operations::adjacentLanelets(laK)) {
            if (std::any_of(simLaneletsP.begin(), simLaneletsP.end(), [adjLet](const std::shared_ptr<Lanelet> &exLet) {
                    return exLet->getId() == adjLet->getId();
                }))
                return true;
        }
    }
    return false;
}

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
                                 return checkSameIncoming(letP, letSim, world->getRoadNetwork());
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